#include "ThreadUtils.h"

#define NOMINMAX
#include <windows.h>

#include <vector>
#include <algorithm>
#include <cstdint>

namespace Ecstasy::Threading
{
    void pinCurrentThreadToCpu(size_t cpuIndex)
    {
        const DWORD_PTR mask = (DWORD_PTR(1) << cpuIndex);
        SetThreadAffinityMask(GetCurrentThread(), mask);
    }

    uint64_t getPcoreAffinityMask()
    {
        uint64_t allCoresMask = 0;
        uint64_t pcoreMask = 0;
        DWORD returnedLength = 0;

        if (!GetLogicalProcessorInformationEx(RelationProcessorCore, nullptr, &returnedLength))
        {
            if (GetLastError() != ERROR_INSUFFICIENT_BUFFER)
                return 0;
        }

        std::vector<BYTE> buffer(returnedLength);

        if (!GetLogicalProcessorInformationEx(
            RelationProcessorCore,
            reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(buffer.data()),
            &returnedLength))
        {
            return 0;
        }

        BYTE* ptr = buffer.data();
        BYTE* end = ptr + returnedLength;

        BYTE maxEfficiency = 0;
        BYTE minEfficiency = 0xFF;
        bool first = true;

        while (ptr < end)
        {
            const SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX* info =
                reinterpret_cast<const SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*>(ptr);

            if (info->Relationship == RelationProcessorCore)
            {
                BYTE eff = info->Processor.EfficiencyClass;
                if (first)
                {
                    minEfficiency = maxEfficiency = eff;
                    first = false;
                }
                else
                {
                    minEfficiency = std::min(minEfficiency, eff);
                    maxEfficiency = std::max(maxEfficiency, eff);
                }

                for (WORD g = 0; g < info->Processor.GroupCount; ++g)
                {
                    allCoresMask |= info->Processor.GroupMask[g].Mask;
                }
            }
            ptr += info->Size;
        }

        // If all cores have the same efficiency, we have a non-hybrid system.
        if (minEfficiency == maxEfficiency)
        {
            return allCoresMask;
        }

        ptr = buffer.data();
        while (ptr < end)
        {
            const SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX* info =
                reinterpret_cast<const SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*>(ptr);

            if (info->Relationship == RelationProcessorCore &&
                info->Processor.EfficiencyClass == maxEfficiency)
            {
                for (WORD g = 0; g < info->Processor.GroupCount; ++g)
                {
                    pcoreMask |= info->Processor.GroupMask[g].Mask;
                }
            }
            ptr += info->Size;
        }

        return pcoreMask;
    }
}