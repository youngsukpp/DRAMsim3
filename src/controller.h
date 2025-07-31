#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <fstream>
#include <map>
#include <unordered_set>
#include <vector>
#include "channel_state.h"
#include "command_queue.h"
#include "common.h"
#include "refresh.h"
#include "simple_stats.h"
#include "pim.h"
#include "pim_common.h"
#include "configuration.h"

#ifdef THERMAL
#include "thermal.h"
#endif  // THERMAL

namespace dramsim3 {

enum class RowBufPolicy { OPEN_PAGE, CLOSE_PAGE, SIZE };

class Controller {
   public:
#ifdef THERMAL
    Controller(int channel, const Config &config, const Timing &timing,
               ThermalCalculator &thermalcalc);
#else
    Controller(int channel, const Config &config, const Timing &timing);
#endif  // THERMAL
    void ClockTick();
    bool WillAcceptTransaction(uint64_t hex_addr, bool is_write, bool trpf);
    bool AddTransaction(Transaction trans);
    int QueueUsage() const;
    // Stats output
    void PrintEpochStats();
    void PrintFinalStats(std::string txt_stats_name);
    void ResetStats() { simple_stats_.Reset(); }
    std::pair<uint64_t, int> ReturnDoneTrans(uint64_t clock);
    std::pair<uint64_t, bool> GetBarrier();
    void UpdatePrefetchTransfer();
    int channel_id_;
    bool CheckAllQueueEmpty();
    bool CheckTotalComplete();

   private:
    uint64_t clk_;
    const Config &config_;
    SimpleStats simple_stats_;
    ChannelState channel_state_;
    CommandQueue cmd_queue_;
    Refresh refresh_;
    int last_cmd_end_clk;
    int overhead_standard_clk;
    bool pim_barrier;

    int total_issued;
    int total_returned;
    uint64_t pf_overhead;
    uint64_t tr_overhead;
    // uint64_t cumul_pf_overhead;
    // uint64_t cumul_tr_overhead;


#ifdef THERMAL
    ThermalCalculator &thermal_calc_;
#endif  // THERMAL

    // queue that takes transactions from CPU side
    bool is_unified_queue_;
    std::vector<Transaction> unified_queue_;
    std::vector<Transaction> read_queue_;
    std::vector<Transaction> write_buffer_;
    std::vector<Transaction> pim_queue_;

    // transactions that are not completed, use map for convenience
    std::multimap<uint64_t, Transaction> pending_rd_q_;
    std::multimap<uint64_t, Transaction> pending_wr_q_;

    // completed transactions
    std::vector<Transaction> return_queue_;
    // completed prefetch / transfer transactions
    std::vector<Transaction> pf_queue_;
    std::vector<Transaction> tr_queue_;

    

    // row buffer policy
    RowBufPolicy row_buf_policy_;

#ifdef CMD_TRACE
    std::ofstream cmd_trace_;
#endif  // CMD_TRACE

    // used to calculate inter-arrival latency
    uint64_t last_trans_clk_;

    // transaction queueing
    int write_draining_;
    void ScheduleTransaction();
    void SchedulePIMTransaction();
    Transaction DecompressPIMInst(Transaction trans, uint64_t clk_, int subvec_idx);
    void IssueCommand(const Command &tmp_cmd);
    Command TransToCommand(const Transaction &trans);
    void UpdateCommandStats(const Command &cmd);
};
}  // namespace dramsim3
#endif
