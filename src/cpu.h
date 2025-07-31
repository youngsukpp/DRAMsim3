#ifndef __CPU_H
#define __CPU_H

#include <fstream>
#include <functional>
#include <random>
#include <string>
#include <list>
#include "memory_system.h"
#include "common.h"
#include "pim.h"
#include <tuple>
// #include "cache_.h"

namespace dramsim3 {

class CPU {
   public:
    CPU(const std::string& config_file, const std::string& output_dir)
        : memory_system_(
              config_file, output_dir,
              std::bind(&CPU::ReadCallBack, this, std::placeholders::_1),
              std::bind(&CPU::WriteCallBack, this, std::placeholders::_1),
              std::bind(&CPU::PIMCallBack, this, std::placeholders::_1)),
          clk_(0) {}
    virtual void ClockTick() = 0;
    void ReadCallBack(uint64_t addr) { return; }
    void WriteCallBack(uint64_t addr) { return; }
    bool PIMCallBack(bool pim_finished) {return pim_finished;}
    void PrintStats(std::string tracename) { memory_system_.PrintStats(tracename); }

   protected:
    MemorySystem memory_system_;
    uint64_t clk_;
};

class RandomCPU : public CPU {
   public:
    using CPU::CPU;
    void ClockTick() override;

   private:
    uint64_t last_addr_;
    bool last_write_ = false;
    std::mt19937_64 gen;
    bool get_next_ = true;
};

class StreamCPU : public CPU {
   public:
    using CPU::CPU;
    void ClockTick() override;

   private:
    uint64_t addr_a_, addr_b_, addr_c_, offset_ = 0;
    std::mt19937_64 gen;
    bool inserted_a_ = false;
    bool inserted_b_ = false;
    bool inserted_c_ = false;
    const uint64_t array_size_ = 2 << 20;  // elements in array
    const int stride_ = 64;                // stride in bytes
};

class TraceBasedCPU : public CPU {
   public:
    TraceBasedCPU(const std::string& config_file, const std::string& output_dir,
                  const std::string& trace_file);
    ~TraceBasedCPU() { trace_file_.close(); }
    void ClockTick() override;

   private:
    std::ifstream trace_file_;
    Transaction trans_;
    bool get_next_ = true;
};


class TraceBasedCPUForHeterogeneousMemory : public CPU {
   public:
    TraceBasedCPUForHeterogeneousMemory(const std::string& config_file_HBM, const std::string& config_file_DIMM, const std::string& output_dir, const std::string& trace_file);
    ~TraceBasedCPUForHeterogeneousMemory() {}

    void PIMCallBack(bool pim_complete) {pim_complete_= pim_complete;}

    float findMedian(std::vector<float> &vec);
    std::pair<float, float> findQuartiles(std::vector<float> &vec);

    int PIMMemGetChannel(uint64_t address);
    int MemGetChannel(uint64_t address);
    void LoadTrace(string filename);
    void PrintStats(std::string tracename) { memory_system_PIM.PrintStats(tracename); }
    void PrintStats_DIMM(std::string tracename) { memory_system_Mem.PrintStats(tracename); }

    void ClockTick();
    int RunPIM();

    void AddBatchTransactions(int batch_idx, int& pool_idx_PIM, int& pool_idx_Mem);
    bool AddTransactionsToPIMMem(int batch_idx, int pool_idx_PIM);
    bool AddTransactionsToMemory(int batch_idx, int pool_idx_Mem);

   private:
    MemorySystem memory_system_PIM;
    MemorySystem memory_system_Mem;
    uint64_t clk_PIM;
    uint64_t clk_Mem;
    const Config* PIMMem_config;
    const Config* Mem_config;

    std::ifstream trace_file_;
    Transaction trans_;
    int complete_transactions = 0;
    int add_cycle = 3;

    std::vector<std::vector<std::tuple<std::string, uint64_t, int>>> PIMMem_transaction;
    std::vector<std::vector<std::tuple<std::string, uint64_t>>> Mem_transaction;

    bool CA_compression;
    bool is_using_vp;
    bool is_using_hp;
    bool is_using_vp_hp;
    bool is_using_PIM;
    std::string PIM_level;
    bool is_using_SRAM;
    bool is_using_hetero;
    int collision;

    int cache_hit_on_transfer_vec=0;
    bool show_lb_ratio = true;

    int channels;
    int ranks;
    int bankgroups;
    int vec_transfers;
    int batch_size;
    std::string addrmapping;
    std::vector<float> batch_max_load;
    std::vector<int> batch_total_embeddings;
    bool pim_complete_ = false;
};

}

#endif