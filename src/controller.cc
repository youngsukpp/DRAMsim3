#include "controller.h"
#include <iomanip>
#include <iostream>
#include <limits>

namespace dramsim3 {

#ifdef THERMAL
Controller::Controller(int channel, const Config &config, const Timing &timing,
                       ThermalCalculator &thermal_calc)
#else
Controller::Controller(int channel, const Config &config, const Timing &timing)
#endif  // THERMAL
    : channel_id_(channel),
      clk_(0),
      config_(config),
      simple_stats_(config_, channel_id_),
      channel_state_(config, timing),
      cmd_queue_(channel_id_, config, channel_state_, simple_stats_),
      refresh_(config, channel_state_),
      pf_overhead(0),
      tr_overhead(0),
    //   cumul_pf_overhead(0),
    //   cumul_tr_overhead(0),
      last_cmd_end_clk(0),
      overhead_standard_clk(0),
      pim_barrier(false),
#ifdef THERMAL
      thermal_calc_(thermal_calc),
#endif  // THERMAL
      is_unified_queue_(config.unified_queue),
      row_buf_policy_(config.row_buf_policy == "CLOSE_PAGE"
                          ? RowBufPolicy::CLOSE_PAGE
                          : RowBufPolicy::OPEN_PAGE),
      last_trans_clk_(0),
      total_issued (0),
      total_returned (0),
      write_draining_(0) {
    if (is_unified_queue_) {
        unified_queue_.reserve(config_.trans_queue_size);
    } else {
        read_queue_.reserve(config_.trans_queue_size);
        write_buffer_.reserve(config_.trans_queue_size);
    }

    if(config_.PIM_enabled)
    {
        pim_queue_.reserve(config_.trans_queue_size);
        pf_queue_.reserve(config_.trans_queue_size);
        tr_queue_.reserve(config_.trans_queue_size);

    }
}

#ifdef CMD_TRACE
    std::string trace_file_name = config_.output_prefix + "ch_" +
                                  std::to_string(channel_id_) + "cmd.trace";
    std::cout << "Command Trace write to " << trace_file_name << std::endl;
    cmd_trace_.open(trace_file_name, std::ofstream::out);
#endif  // CMD_TRACE

std::pair<uint64_t, int> Controller::ReturnDoneTrans(uint64_t clk) {
    auto it = return_queue_.begin();

    while (it != return_queue_.end()) {
        bool empty = return_queue_.empty() && pending_rd_q_.empty() && pim_queue_.empty();
        // std::cout << "Queue Empty : " << empty  << " " << pending_rd_q_.size() << " " << pim_queue_.size() << " " << return_queue_.size() << std::endl;
        // std::cout << "Cycle : " << clk << " " << it->complete_cycle << "  " << it->addr  << std::endl;

        if (clk >= it->complete_cycle) {
            if (it->is_write) {
                simple_stats_.Increment("num_writes_done");
            } else {
                simple_stats_.Increment("num_reads_done");
                simple_stats_.AddValue("read_latency", clk_ - it->added_cycle);
            }
            auto pair = std::make_pair(it->addr, it->is_write);
            it = return_queue_.erase(it);
            last_cmd_end_clk = it->complete_cycle;
            total_returned++;
            // std::cout << total_returned << std::endl;
            return pair;
        } else {
            ++it;
        }
    }
    return std::make_pair(-1, -1);
}

bool Controller::CheckTotalComplete() {
    // std::cout << "issued, returned : " << total_issued << " " << total_returned << std::endl;
    return total_issued == total_returned;
}

bool Controller::CheckAllQueueEmpty() {
    return pim_queue_.empty() && pending_rd_q_.empty() && return_queue_.empty() && read_queue_.empty();
}

void Controller::ClockTick() {
    // update refresh counter
    refresh_.ClockTick();

    bool cmd_issued = false;
    Command cmd;
    if (channel_state_.IsRefreshWaiting()) {
        cmd = cmd_queue_.FinishRefresh();
    }

    // cannot find a refresh related command or there's no refresh
    if(config_.PIM_enabled)
    {
        if (!cmd.IsValid()) {
            for(int i=0; i<config_.ranks; i++)
            {
                if(config_.PIM_level == "rank")
                {
                    cmd = cmd_queue_.RankPIM_GetCommandToIssue(i);
                    if (cmd.IsValid()) {
                        IssueCommand(cmd);
                        cmd_issued = true;
                    }                    
                }
                else if(config_.PIM_level == "bankgroup")
                {
                    int k = 0;
                    for(int j=0; j<config_.bankgroups;j++)
                    {
                        cmd = cmd_queue_.BGPIM_GetCommandToIssue(i, j);
                        if (cmd.IsValid()) {
                            IssueCommand(cmd);
                            cmd_issued = true;
                            k++;
                        }
                    }
                    // std::cout << k << std::endl;
                    
                }
                else if (config_.PIM_level == "bank")
                {
                    for (int j=0; j<config_.bankgroups;j++)
                    {
                        for (int k=0; k<config_.banks_per_group; k++)
                        {
                            cmd = cmd_queue_.BankPIM_GetCommandToIssue(i,j,k);
                            if (cmd.IsValid()) {
                                IssueCommand(cmd);
                                cmd_issued = true;
                            }
                        }
                    }
                }
            }
        }
        else
        {
            IssueCommand(cmd);
            cmd_issued = true;
        }
    }
    else
    {
        if (!cmd.IsValid())
            cmd = cmd_queue_.GetCommandToIssue();
        if (cmd.IsValid()) {
            IssueCommand(cmd);
            cmd_issued = true;
            if (config_.enable_hbm_dual_cmd) {
                Command second_cmd;
                second_cmd = cmd_queue_.GetSecondCommandToIssue();

                if (second_cmd.IsValid()) {
                    if (second_cmd.IsReadWrite() != cmd.IsReadWrite()) {
                        if(second_cmd.IsReadWrite())
                            cmd_queue_.EraseSecondRWCommand(second_cmd);
                        IssueCommand(second_cmd);
                        simple_stats_.Increment("hbm_dual_cmds");
                    }
                }
            }
        }
    }

    // power updates pt 1
    for (int i = 0; i < config_.ranks; i++) {
        if (channel_state_.IsRankSelfRefreshing(i)) {
            simple_stats_.IncrementVec("sref_cycles", i);
        } else {
            bool all_idle = channel_state_.IsAllBankIdleInRank(i);
            if (all_idle) {
                simple_stats_.IncrementVec("all_bank_idle_cycles", i);
                channel_state_.rank_idle_cycles[i] += 1;
            } else {
                simple_stats_.IncrementVec("rank_active_cycles", i);
                // reset
                channel_state_.rank_idle_cycles[i] = 0;
            }
        }
    }

    // power updates pt 2: move idle ranks into self-refresh mode to save power
    if (config_.enable_self_refresh && !cmd_issued) {
        for (auto i = 0; i < config_.ranks; i++) {
            if (channel_state_.IsRankSelfRefreshing(i)) {
                // wake up!
                if (!cmd_queue_.rank_q_empty[i]) {
                    auto addr = Address();
                    addr.rank = i;
                    auto cmd = Command(CommandType::SREF_EXIT, addr, -1);
                    cmd = channel_state_.GetReadyCommand(cmd, clk_);
                    if (cmd.IsValid()) {
                        IssueCommand(cmd);
                        break;
                    }
                }
            } else {
                if (cmd_queue_.rank_q_empty[i] &&
                    channel_state_.rank_idle_cycles[i] >=
                        config_.sref_threshold) {
                    auto addr = Address();
                    addr.rank = i;
                    auto cmd = Command(CommandType::SREF_ENTER, addr, -1);
                    cmd = channel_state_.GetReadyCommand(cmd, clk_);
                    if (cmd.IsValid()) {
                        IssueCommand(cmd);
                        break;
                    }
                }
            }
        }
    }

    if(config_.PIM_enabled)
    {
        SchedulePIMTransaction();
        // if(pim_barrier)
        //     UpdatePrefetchTransfer();
    }
    else
        ScheduleTransaction();

    clk_++;
    cmd_queue_.ClockTick();
    simple_stats_.Increment("num_cycles");
    return;
}

bool Controller::WillAcceptTransaction(uint64_t hex_addr, bool is_write, bool trpf)  {
    // if(config_.PIM_enabled)
    // {
    //     bool trpf_processing = !pf_queue_.empty() || !tr_queue_.empty() || !pim_queue_.empty() || !pending_rd_q_.empty();
    //     bool rd_processing = !return_queue_.empty() || !pim_queue_.empty() || !pending_rd_q_.empty();

    //     if(!trpf)
    //     {
    //         if(trpf_processing)
    //             return false;            
    //         else
    //         {
    //             if(pim_barrier)
    //                 pim_barrier = false;
    //             return true;                
    //         }
    //     }
    //     else
    //     {
    //         if(rd_processing)
    //             return false;
    //         else
    //         {
    //             pim_barrier=true; 
    //             return true;
    //         }
    //     }

    // }
    
    if (is_unified_queue_) {
        return unified_queue_.size() < unified_queue_.capacity();
    } else if (!is_write) {
        return read_queue_.size() < read_queue_.capacity();
    } else {
        return write_buffer_.size() < write_buffer_.capacity();
    }
}

bool Controller::AddTransaction(Transaction trans) {
    trans.added_cycle = clk_;
    simple_stats_.AddValue("interarrival_latency", clk_ - last_trans_clk_);
    last_trans_clk_ = clk_;

    if (trans.is_write) {
        if (pending_wr_q_.count(trans.addr) == 0) {  // can not merge writes
            pending_wr_q_.insert(std::make_pair(trans.addr, trans));
            if (is_unified_queue_) {
                unified_queue_.push_back(trans);
            } else {
                write_buffer_.push_back(trans);
            }
        }
        trans.complete_cycle = clk_ + 1;
        return_queue_.push_back(trans);
        return true;
    } else {  // read
        // if in write buffer, use the write buffer value
        if (pending_wr_q_.count(trans.addr) > 0) {
            trans.complete_cycle = clk_ + 1;
            return_queue_.push_back(trans);
            return true;
        }

        if(config_.PIM_enabled) {
            trans.pim_values.skewed_cycle = clk_ + trans.pim_values.skewed_cycle;
            pim_queue_.push_back(trans);
        }
        else {
            pending_rd_q_.insert(std::make_pair(trans.addr, trans));

            if (pending_rd_q_.count(trans.addr) == 1) {
                if (is_unified_queue_) {
                    unified_queue_.push_back(trans);
                } else {
                    read_queue_.push_back(trans);
                }
            }
        }
        return true;
    }
}

void Controller::UpdatePrefetchTransfer(){

    auto pf_it = pf_queue_.begin();
    while(pf_it != pf_queue_.end())
    {
        if(pf_it->complete_cycle <= clk_)
        {
            pf_it = pf_queue_.erase(pf_it);
            break;
        }
        else
            ++pf_it;
    }

    auto tr_it = tr_queue_.begin();
    while(tr_it != tr_queue_.end())
    {
        if(tr_it->complete_cycle <= clk_)
        {
            tr_it = tr_queue_.erase(tr_it);
            break;
        }
        else
            ++tr_it;
    }

}

Transaction Controller::DecompressPIMInst(Transaction trans, uint64_t clk_, int subvec_idx)
{
    Transaction sub_vec_trans = trans;
    sub_vec_trans.addr = trans.addr - 64*subvec_idx; 

    sub_vec_trans.pim_values.skewed_cycle = clk_ + config_.skewed_cycle;
    sub_vec_trans.pim_values.decode_cycle = clk_ + config_.decode_cycle;

    return sub_vec_trans;
}

// Interaction between MC extension and PIM units
// Command queue is utilized as a part of the PIM queue to meet the compatibility of DRAMSim3
void Controller::SchedulePIMTransaction(){

    // std::cout << return_queue_.size() << " " << clk_ << std::endl;

    for (auto it = pim_queue_.begin(); it != pim_queue_.end(); it++) {

        int total_pims = (config_.PIM_level == "rank") ? config_.ranks : config_.bankgroups;
        
        if(it->pim_values.transfer_cmd) {
            it->complete_cycle = clk_ + it->pim_values.vlen * config_.tCCD_S * total_pims;
            tr_queue_.push_back(*it);
            pim_queue_.erase(it);
            break;
        }
        // if(config_.vp_mapping)
        // {
        //     for(int i=0; i<total_pims; i++)
        //     {
        //         Transaction v_trans = *it;
        //         Address addr = config_.AddressMapping(v_trans.addr);

        //         if(config_.PIM_level == "rank")
        //             v_trans.addr = config_.GenerateAddress(addr.channel, i, addr.bankgroup, addr.bank, addr.row, addr.column);
        //         else
        //             v_trans.addr = config_.GenerateAddress(addr.channel, addr.rank, i, addr.bank, addr.row, addr.column);

        //         for(int j=0; j<it->pim_values.vlen; j++)
        //         {
        //             Transaction sub_trans = DecompressPIMInst(v_trans, clk_, j);
        //             auto cmd = TransToCommand(sub_trans);
        //             if (cmd_queue_.WillAcceptCommand(cmd.Rank(), cmd.Bankgroup(),
        //                                             cmd.Bank())) {
        //                 cmd_queue_.AddCommand(cmd);
        //                 pending_rd_q_.insert(std::make_pair(sub_trans.addr, sub_trans));

        //             }
        //         }
        //     }
        //     pim_queue_.erase(it);
        //     break;
        // }
        if(it->pim_values.prefetch_cmd)
        {
            for(int i=0; i<total_pims; i++)
            {
                Transaction p_trans = *it;
                Address addr = config_.AddressMapping_2nd(p_trans.addr);

                if(config_.PIM_level == "rank")
                    p_trans.addr = config_.GenerateAddress_2nd(addr.channel, i, addr.bankgroup, addr.bank, addr.row, addr.column);
                else
                    p_trans.addr = config_.GenerateAddress_2nd(addr.channel, addr.rank, i, addr.bank, addr.row, addr.column);

                for(int j=0; j<it->pim_values.vlen; j++)
                {
                    Transaction sub_trans = DecompressPIMInst(p_trans, clk_, j);
                    auto cmd = TransToCommand(sub_trans);
                    if (cmd_queue_.WillAcceptCommand(cmd.Rank(), cmd.Bankgroup(),
                                                    cmd.Bank())) {
                        cmd_queue_.AddCommand(cmd);
                        pending_rd_q_.insert(std::make_pair(sub_trans.addr, sub_trans));
                        total_issued++;
                    }
                }
            }            
            pim_queue_.erase(it);
            break;
        }
        else if(it->pim_values.deliver_cmd)
        {
            if(it->pim_values.skewed_cycle < clk_)
            {
                for(int i=0; i<it->pim_values.vlen; i++)
                {
                    Transaction sub_trans = DecompressPIMInst(*it, clk_, i);
                    auto cmd = TransToCommand(sub_trans);
                    if (cmd_queue_.WillAcceptCommand(cmd.Rank(), cmd.Bankgroup(),
                                                    cmd.Bank())) {
                        cmd_queue_.AddCommand(cmd);
                        pending_rd_q_.insert(std::make_pair(sub_trans.addr, sub_trans));
                        total_issued++;
                    }
                }
                pim_queue_.erase(it);
                break;
            }
        }
        else
        {
            for(int i=0; i<it->pim_values.vlen; i++)
            {
                // std::cout << i << std::endl;
                Transaction sub_trans = DecompressPIMInst(*it, clk_, i);
                auto cmd = TransToCommand(sub_trans);
                if (cmd_queue_.WillAcceptCommand(cmd.Rank(), cmd.Bankgroup(),
                                                cmd.Bank())) {
                    cmd_queue_.AddCommand(cmd);
                    pending_rd_q_.insert(std::make_pair(sub_trans.addr, sub_trans));
                    total_issued++;

                }
            }
            pim_queue_.erase(it);
            break;
        }
    }
}

void Controller::ScheduleTransaction() {
    // determine whether to schedule read or write
    if (write_draining_ == 0 && !is_unified_queue_) {
        // we basically have a upper and lower threshold for write buffer
        if ((write_buffer_.size() >= write_buffer_.capacity()) ||
            (write_buffer_.size() > 8 && cmd_queue_.QueueEmpty())) {
            write_draining_ = write_buffer_.size();
        }
    }

    std::vector<Transaction> &queue =
        is_unified_queue_ ? unified_queue_
                          : write_draining_ > 0 ? write_buffer_ : read_queue_;

    for (auto it = queue.begin(); it != queue.end(); it++) {
        auto cmd = TransToCommand(*it);
        if (cmd_queue_.WillAcceptCommand(cmd.Rank(), cmd.Bankgroup(),
                                        cmd.Bank())) {
            if (!is_unified_queue_ && cmd.IsWrite()) {
                // Enforce R->W dependency
                if (pending_rd_q_.count(it->addr) > 0) {
                    write_draining_ = 0;
                    break;
                }
                write_draining_ -= 1;
            }
            cmd_queue_.AddCommand(cmd);
            queue.erase(it);
            total_issued++;
            break;
            
        }
    }
}

void Controller::IssueCommand(const Command &cmd) {
#ifdef CMD_TRACE
    cmd_trace_ << std::left << std::setw(18) << clk_ << " " << cmd << std::endl;
#endif  // CMD_TRACE
#ifdef THERMAL
    // add channel in, only needed by thermal module
    thermal_calc_.UpdateCMDPower(channel_id_, cmd, clk_);
#endif  // THERMAL
    // if read/write, update pending queue and return queue
    if (cmd.IsRead()) {
        auto num_reads = pending_rd_q_.count(cmd.hex_addr);
        if (num_reads == 0) {
            std::cerr << cmd.hex_addr << " not in read queue! " << std::endl;
            exit(1);
        }

        // if there are multiple reads pending return them all
        // while (num_reads > 0) {
            auto it = pending_rd_q_.find(cmd.hex_addr);
            it->second.complete_cycle = clk_ + config_.read_delay;
            if(config_.PIM_enabled) {
                bool pf = it->second.pim_values.prefetch_cmd;
                if(!pf)
                    return_queue_.push_back(it->second);
                else
                    pf_queue_.push_back(it->second);
                pending_rd_q_.erase(it);
            }
            else
            {
                return_queue_.push_back(it->second);
                pending_rd_q_.erase(it);
                num_reads -= 1;                
            }

            // num_reads -= 1;
        // }

    } else if (cmd.IsWrite()) {
        // there should be only 1 write to the same location at a time
        auto it = pending_wr_q_.find(cmd.hex_addr);
        if (it == pending_wr_q_.end()) {
            std::cerr << cmd.hex_addr << " not in write queue!" << std::endl;
            exit(1);
        }
        auto wr_lat = clk_ - it->second.added_cycle + config_.write_delay;
        simple_stats_.AddValue("write_latency", wr_lat);
        pending_wr_q_.erase(it);
    }
    // must update stats before states (for row hits)
    // std::cout << "----- processing in controller -----" << std::endl;
    // std::cout << "\t Issuing addr : " << cmd.hex_addr << " in channel : " << channel_id_ << std::endl;
    // std::cout << "------------------------------\n" << std::endl;

    UpdateCommandStats(cmd);
    channel_state_.UpdateTimingAndStates(cmd, clk_);
}

Command Controller::TransToCommand(const Transaction &trans) {
    auto addr = config_.AddressMapping(trans.addr);
    if(config_.PIM_enabled && (trans.pim_values.read_dup_cmd || trans.pim_values.prefetch_cmd))
        addr = config_.AddressMapping_2nd(trans.addr);
    CommandType cmd_type;
    if (row_buf_policy_ == RowBufPolicy::OPEN_PAGE) {
        cmd_type = trans.is_write ? CommandType::WRITE : CommandType::READ;
    } else {
        cmd_type = trans.is_write ? CommandType::WRITE_PRECHARGE
                                  : CommandType::READ_PRECHARGE;
    }
    return Command(cmd_type, addr, trans.addr);
}

int Controller::QueueUsage() const { return cmd_queue_.QueueUsage(); }

void Controller::PrintEpochStats() {
    simple_stats_.Increment("epoch_num");
    simple_stats_.PrintEpochStats();
#ifdef THERMAL
    for (int r = 0; r < config_.ranks; r++) {
        double bg_energy = simple_stats_.RankBackgroundEnergy(r);
        thermal_calc_.UpdateBackgroundEnergy(channel_id_, r, bg_energy);
    }
#endif  // THERMAL
    return;
}

void Controller::PrintFinalStats(std::string txt_stats_name) {
    simple_stats_.PrintFinalStats(txt_stats_name);
    // std::cout << "Overhead on channel " << channel_id_ << " - prefech/transfer : " << cumul_pf_overhead << "/" << cumul_tr_overhead << std::endl;


#ifdef THERMAL
    for (int r = 0; r < config_.ranks; r++) {
        double bg_energy = simple_stats_.RankBackgroundEnergy(r);
        thermal_calc_.UpdateBackgroundEnergy(channel_id_, r, bg_energy);
    }
#endif  // THERMAL
    return;
}

void Controller::UpdateCommandStats(const Command &cmd) {
    switch (cmd.cmd_type) {
        case CommandType::READ:
        case CommandType::READ_PRECHARGE:
            simple_stats_.Increment("num_read_cmds");
            if (channel_state_.RowHitCount(cmd.Rank(), cmd.Bankgroup(),
                                           cmd.Bank()) != 0) {
                simple_stats_.Increment("num_read_row_hits");
            }
            break;
        case CommandType::WRITE:
        case CommandType::WRITE_PRECHARGE:
            simple_stats_.Increment("num_write_cmds");
            if (channel_state_.RowHitCount(cmd.Rank(), cmd.Bankgroup(),
                                           cmd.Bank()) != 0) {
                simple_stats_.Increment("num_write_row_hits");
            }
            break;
        case CommandType::ACTIVATE:
            simple_stats_.Increment("num_act_cmds");
            break;
        case CommandType::PRECHARGE:
            simple_stats_.Increment("num_pre_cmds");
            break;
        case CommandType::REFRESH:
            simple_stats_.Increment("num_ref_cmds");
            break;
        case CommandType::REFRESH_BANK:
            simple_stats_.Increment("num_refb_cmds");
            break;
        case CommandType::SREF_ENTER:
            simple_stats_.Increment("num_srefe_cmds");
            break;
        case CommandType::SREF_EXIT:
            simple_stats_.Increment("num_srefx_cmds");
            break;
        default:
            AbruptExit(__FILE__, __LINE__);
    }
}

}  // namespace dramsim3