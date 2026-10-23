#pragma once
#include <cstdint>
#include <span>
#include <vector>

namespace Ecstasy::Core::Threading
{
    // Which cores are eligible.
    enum class CoreSet
    {
        All,
        Performance // P-cores on hybrid CPUs; same as All on non-hybrid CPUs.
    };

    // Physical = one thread per core. Logical = SMT siblings allowed (used after all physical cores).
    enum class SmtMode
    {
        Logical,
        Physical
    };

    // Type of the core a worker runs on. Unknown = unpinned worker.
    enum class CoreType : uint8_t
    {
        Performance, // Also used for every core on non-hybrid CPUs.
        Efficiency,
        Unknown
    };

    enum class ThreadPriority
    {
        Idle,
        Lowest,
        BelowNormal,
        Normal,
        AboveNormal,
        Highest,
        TimeCritical
    };

    struct CpuInfo
    {
        uint32_t id = 0;              // Flat id: group * 64 + index.
        uint16_t group = 0;           // Windows processor group.
        uint8_t  index = 0;           // Bit inside the group mask.
        uint8_t  efficiencyClass = 0;
        uint8_t  smtIndex = 0;        // 0 = first thread of its physical core.
        uint32_t coreId = 0;          // Physical core ordinal.
        bool     performance = true;
        bool     allowed = true;      // Within process affinity.
    };

    struct CpuTopology
    {
        std::vector<CpuInfo> cpus;    // Sorted by id.
        size_t physicalCoreCount = 0;
        bool hybrid = false;
    };

    // Queried once, cached.
    const CpuTopology& getCpuTopology();

    // Ordered by preference: physical cores first (best class first), SMT siblings after.
    std::vector<CpuInfo> selectCpus(CoreSet coreSet, SmtMode smtMode);

    // Resolves flat ids. Unknown/disallowed ids are skipped and counted in 'invalid'.
    std::vector<CpuInfo> resolveCpus(std::span<const uint32_t> ids, size_t* invalid = nullptr);

    bool pinCurrentThreadToCpu(const CpuInfo& cpu);
    bool setThreadPriority(ThreadPriority priority);

    // Console overview of the topology. 'pinned[i]' = CPU of worker i (optional).
    void printCpuTopology(std::span<const CpuInfo> pinned = {});
}