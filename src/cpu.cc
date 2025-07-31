#include "cpu.h"
#include <sstream>
#include <string>
#include <chrono>
#include <ctime>
#include <algorithm>

namespace dramsim3 {

float TraceBasedCPUForHeterogeneousMemory::findMedian(std::vector<float> &vec) {
    size_t size = vec.size();
    std::sort(vec.begin(), vec.end());

    if (size % 2 == 0) {
        return (vec[size / 2 - 1] + vec[size / 2]) / 2.0;
    } else {
        return vec[size / 2];
    }
}

std::pair<float, float> TraceBasedCPUForHeterogeneousMemory::findQuartiles(std::vector<float> &vec) {
    size_t size = vec.size();
    size_t mid = size / 2;

    std::vector<float> lower_half(vec.begin(), vec.begin() + mid);
    std::vector<float> upper_half(vec.begin() + (size % 2 == 0 ? mid : mid + 1), vec.end());

    float q1 = findMedian(lower_half);
    float q3 = findMedian(upper_half);

    return {q1, q3};
}

void RandomCPU::ClockTick() {
    // Create random CPU requests at full speed
    // this is useful to exploit the parallelism of a DRAM protocol
    // and is also immune to address mapping and scheduling policies
    memory_system_.ClockTick();
    if (get_next_) {
        last_addr_ = gen();
        last_write_ = (gen() % 3 == 0);
    }
    get_next_ = memory_system_.WillAcceptTransaction(last_addr_, last_write_, false);
    if (get_next_) {
        PimValues pim_values;
        memory_system_.AddTransaction(last_addr_, last_write_, pim_values);
    }
    clk_++;
    return;
}

void StreamCPU::ClockTick() {
    // stream-add, read 2 arrays, add them up to the third array
    // this is a very simple approximate but should be able to produce
    // enough buffer hits

    // moving on to next set of arrays
    memory_system_.ClockTick();
    if (offset_ >= array_size_ || clk_ == 0) {
        addr_a_ = gen();
        addr_b_ = gen();
        addr_c_ = gen();
        offset_ = 0;
    }
    PimValues pim_values;
    if (!inserted_a_ &&
        memory_system_.WillAcceptTransaction(addr_a_ + offset_, false, false)) {
        memory_system_.AddTransaction(addr_a_ + offset_, false, pim_values);
        inserted_a_ = true;
    }
    if (!inserted_b_ &&
        memory_system_.WillAcceptTransaction(addr_b_ + offset_, false, false)) {
        memory_system_.AddTransaction(addr_b_ + offset_, false, pim_values);
        inserted_b_ = true;
    }
    if (!inserted_c_ &&
        memory_system_.WillAcceptTransaction(addr_c_ + offset_, true, false)) {
        memory_system_.AddTransaction(addr_c_ + offset_, true, pim_values);
        inserted_c_ = true;
    }
    // moving on to next element
    if (inserted_a_ && inserted_b_ && inserted_c_) {
        offset_ += stride_;
        inserted_a_ = false;
        inserted_b_ = false;
        inserted_c_ = false;
    }
    clk_++;
    return;
}

TraceBasedCPU::TraceBasedCPU(const std::string& config_file,
                             const std::string& output_dir,
                             const std::string& trace_file)
    : CPU(config_file, output_dir) {
    trace_file_.open(trace_file);
    if (trace_file_.fail()) {
        std::cerr << "Trace file does not exist" << std::endl;
        AbruptExit(__FILE__, __LINE__);
    }
}

void TraceBasedCPU::ClockTick() {
    memory_system_.ClockTick();
    if (!trace_file_.eof()) {
        if (get_next_) {
            get_next_ = false;
            trace_file_ >> trans_;
        }
        if (trans_.added_cycle <= clk_) {
            get_next_ = memory_system_.WillAcceptTransaction(trans_.addr,
                                                             trans_.is_write, false);
            if (get_next_) {
                PimValues pim_values;
                memory_system_.AddTransaction(trans_.addr, trans_.is_write, pim_values);
            }
        }
    }
    clk_++;
    return;
}

TraceBasedCPUForHeterogeneousMemory::TraceBasedCPUForHeterogeneousMemory(const std::string& config_file_HBM, 
                                                                            const std::string& config_file_DIMM, 
                                                                            const std::string& output_dir,
                                                                            const std::string& trace_file)
    : CPU(config_file_HBM, output_dir),
    memory_system_PIM(config_file_HBM, output_dir,
        std::bind(&TraceBasedCPUForHeterogeneousMemory::ReadCallBack, this, std::placeholders::_1),
        std::bind(&TraceBasedCPUForHeterogeneousMemory::WriteCallBack, this, std::placeholders::_1),
        std::bind(&TraceBasedCPUForHeterogeneousMemory::PIMCallBack, this, std::placeholders::_1)),
    memory_system_Mem(config_file_DIMM, output_dir,
            std::bind(&TraceBasedCPUForHeterogeneousMemory::ReadCallBack, this, std::placeholders::_1),
            std::bind(&TraceBasedCPUForHeterogeneousMemory::WriteCallBack, this, std::placeholders::_1),
            std::bind(&TraceBasedCPUForHeterogeneousMemory::PIMCallBack, this, std::placeholders::_1)),
    clk_PIM(0),
    clk_Mem(0)
{
    PIMMem_config = memory_system_PIM.config_copy;
    Mem_config = memory_system_Mem.config_copy;

    channels = PIMMem_config->channels;
    ranks = PIMMem_config->ranks;
    bankgroups = PIMMem_config->bankgroups;

    is_using_PIM = PIMMem_config->PIM_enabled;
    PIM_level = PIMMem_config->PIM_level;
    is_using_SRAM = PIMMem_config->SRAM_enabled;
    batch_size = PIMMem_config->batch_size;
    addrmapping = PIMMem_config->address_mapping;
    CA_compression = PIMMem_config->CA_compression;

    auto time_now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()); 

    std::cout << "\nsimulation start time : " <<  ctime(&time_now) << std::endl;
    std::cout << "------------ Info ------------" << std::endl;
    std::cout << "- PIM level : " << is_using_PIM << std::endl;
    std::cout << "- SRAM cache : " << is_using_SRAM << std::endl;

    if(config_file_HBM.find("TensorDIMM") != std::string::npos)
        is_using_vp = true;
    else if(config_file_HBM.find("ProactivePIM") != std::string::npos)
        is_using_vp_hp = true;
    else
        is_using_hp = true;

    // if((config_file_HBM.find("ProactivePIM") || config_file_HBM.find("SPACE")) != std::string::npos)
    //     is_using_hetero = true;

    int total_cycle = 0;
    LoadTrace(trace_file);
    std::cout << "------- simulation start --------" << std::endl;
    total_cycle = RunPIM();
    std::cout << "- total cycle : " << total_cycle << std::endl;

    std::string trace_name = trace_file;
    std::string erase_1 = "./traces/kaggle/";
    std::string erase_2 = ".txt";
    size_t pos_1 = trace_name.find(erase_1);
    if(pos_1 != std::string::npos)
        trace_name.erase(pos_1, erase_1.size());

    size_t pos_2 = trace_name.find(erase_2);
    if(pos_2 != std::string::npos)
        trace_name.erase(pos_2, erase_2.size());

    std::cout << trace_name << std::endl;
    std::string HBM_stat = trace_name + "_HBM";
    std::string DIMM_stat = trace_name + "_DIMM";
    PrintStats(HBM_stat);
    PrintStats_DIMM(DIMM_stat);
}

int TraceBasedCPUForHeterogeneousMemory::RunPIM() {
    int i = 0;
    int total_batch = (int)PIMMem_transaction.size();
    for(int i=0; i<total_batch; i++)
    {
        int pool_idx_pim = 0;
        int pool_idx_mem = 0;
        int poolings_pim = PIMMem_transaction[i].size();
        int poolings_mem = Mem_transaction[i].size();

        // if(i%100 == 0)
        std::cout << i << " / " << total_batch << " - cycles : " << clk_PIM << std::endl;
        while(pool_idx_pim < poolings_pim) // && pool_idx_mem < poolings_mem)
        {
            ClockTick();
            AddBatchTransactions(i, pool_idx_pim, pool_idx_mem);
        }
        while(!pim_complete_)
        {
            ClockTick();
        }

    }

    return clk_PIM;
}

void TraceBasedCPUForHeterogeneousMemory::AddBatchTransactions(int batch_idx, int& pool_idx_PIM, int& pool_idx_Mem)
{
    bool success = AddTransactionsToPIMMem(batch_idx, pool_idx_PIM);
    if(success)
        pool_idx_PIM++;

    if(is_using_hetero)
    {
        std::cout << "mem" << std::endl;
        bool success = AddTransactionsToMemory(batch_idx, pool_idx_Mem);
        if(success)
            pool_idx_Mem++;
        std::cout << "mem complete" << std::endl;
    }
}

bool TraceBasedCPUForHeterogeneousMemory::AddTransactionsToPIMMem(int batch_idx, int pool_idx)
{
    std::string cmd = std::get<0>(PIMMem_transaction[batch_idx][pool_idx]);
    uint64_t addr = std::get<1>(PIMMem_transaction[batch_idx][pool_idx]);
    int vlen = std::get<2>(PIMMem_transaction[batch_idx][pool_idx]);
    bool prefetch = cmd.compare("PR") == 0 ? true : false;
    bool transfer = cmd.compare("TR") == 0 ? true : false;
    bool readall = cmd.compare("RDD") == 0 ? true : false;
    bool deliver = cmd.compare("DR") == 0 ? true : false;
    
    int skewed_cycle = 0;
    if(deliver)
        skewed_cycle = vlen * PIMMem_config->burst_cycle;
    // int curr_channel = PIMMemGetChannel(addr);
    
    // if(transfer)
    //     std::cout << prefetch << " " << transfer << readall << std::endl;

    bool PIM_mem_get_next_ = false;
    if(transfer || prefetch)
    {
        for(int i=0; i<channels; i++)
        {
            Address addrmap = PIMMem_config->AddressMapping(addr);
            uint64_t ch_addr = PIMMem_config->GenerateAddress(i, addrmap.rank, addrmap.bankgroup, addrmap.bank, addrmap.row, addrmap.column);
            PIM_mem_get_next_ = memory_system_PIM.WillAcceptTransaction(ch_addr, false, transfer || prefetch);
            if(!PIM_mem_get_next_)
                break;
        }
    }   

    PIM_mem_get_next_ = memory_system_PIM.WillAcceptTransaction(addr, false, prefetch || transfer);

    if (PIM_mem_get_next_) {
        PimValues pim_values(vlen, prefetch, transfer, readall, deliver, skewed_cycle, 0);
        // PIMMem_address_in_processing.push_back(emb_data.target_addr);
        if(transfer || prefetch)
        {
            // std::cout << "trpf" << std::endl;
            for(int i=0; i<channels; i++)
            {
                Address addrmap = PIMMem_config->AddressMapping(addr);
                uint64_t ch_addr = PIMMem_config->GenerateAddress(i, addrmap.rank, addrmap.bankgroup, addrmap.bank, addrmap.row, addrmap.column);
                memory_system_PIM.AddTransaction(ch_addr, false, pim_values);
            }
        }
        else
        {
            memory_system_PIM.AddTransaction(addr, false, pim_values);
        }

        return true;
    }
    return false;
}

bool TraceBasedCPUForHeterogeneousMemory::AddTransactionsToMemory(int batch_idx, int pool_idx)
{
    uint64_t target_addr = std::get<1>(Mem_transaction[batch_idx][pool_idx]);
    bool Mem_get_next_ = memory_system_Mem.WillAcceptTransaction(target_addr, false, false);
    int curr_channel = MemGetChannel(target_addr);

    if (Mem_get_next_) {
        PimValues pim_values;
        memory_system_Mem.AddTransaction(target_addr, false, pim_values);
        // Mem_address_in_processing.push_back(target_addr);
        return true;
    }
    return false;
}

void TraceBasedCPUForHeterogeneousMemory::ClockTick(){
    memory_system_PIM.ClockTick();
    // memory_system_Mem.ClockTick();
    // clk_Mem++;
    // // calculated considering clock frequency difference
    if(clk_PIM % 2 == 0)
    {
        memory_system_Mem.ClockTick();
        memory_system_Mem.ClockTick();
        memory_system_Mem.ClockTick();
        clk_Mem = clk_Mem + 3;
    }    
    clk_PIM++;
}

int TraceBasedCPUForHeterogeneousMemory::PIMMemGetChannel(uint64_t address) {
    Address addr = PIMMem_config->AddressMapping(address);
    return addr.channel;
}

int TraceBasedCPUForHeterogeneousMemory::MemGetChannel(uint64_t address) {
    Address addr = Mem_config->AddressMapping(address);
    return addr.channel;
}

void TraceBasedCPUForHeterogeneousMemory::LoadTrace(string filename)
{
    std::string line, str_tmp;
    std::ifstream file(filename);
    std::stringstream ss;
    int count = 0;

    std::vector<std::tuple<std::string, uint64_t, int>> HBM_pooling;
    std::vector<std::tuple<std::string, uint64_t>> DIMM_pooling;
    PIMMem_transaction.push_back(HBM_pooling);
    Mem_transaction.push_back(DIMM_pooling);
    PIMMem_transaction[count].clear();
    Mem_transaction[count].clear();

    // std::cout << "- trace file : " << filename << std::endl;

    if(file.is_open())
    {
        while (std::getline(file, line, '\n'))
        {

            if(line.empty())
            {
                count++;
                std::vector<std::tuple<std::string, uint64_t, int>> HBM_pooling;
                std::vector<std::tuple<std::string, uint64_t>> DIMM_pooling;
                PIMMem_transaction.push_back(HBM_pooling);
                Mem_transaction.push_back(DIMM_pooling);
                PIMMem_transaction[count].clear();
                Mem_transaction[count].clear();
                continue;
            }

            ss.str(line);
            std::string mode = "DIMM";
            std::string cmd;
            uint64_t addr = 0;
            int vlen = 0;

            int str_count = 0;
            while (ss >> str_tmp)
            {
                if(str_count == 0)
                {
                    if(str_tmp == "\n")
                        count++;
                    else
                        mode = str_tmp;
                }
                else if(str_count == 1)
                    cmd = str_tmp;
                else if(str_count == 2)
                    addr = std::stoull(str_tmp);
                else if(str_count == 3)
                    vlen = std::stoull(str_tmp);
                str_count++;
            }

            if(is_using_hetero)
            {
                if(mode == "HBM")
                {
                    PIMMem_transaction[count].push_back(std::make_tuple(cmd, addr, vlen));
                    // if(count==0)
                    //     std::cout << cmd << " " PIMMem_transaction[count].size() << std::endl;
                }
                else
                    Mem_transaction[count].push_back(std::make_tuple(cmd, addr));
            }
            else
                PIMMem_transaction[count].push_back(std::make_tuple(cmd, addr, vlen));

            ss.clear();
        }
    }
}




}  // namespace dramsim3


