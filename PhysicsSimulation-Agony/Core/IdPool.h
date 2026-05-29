#pragma once
#include <optional>
#include <set>
#include <type_traits>
#include <iterator>
#include <limits>
#include <stdexcept>

template <typename T>
class IdPool
{
    static_assert(std::is_integral_v<T>, "T must be an integral type");

    struct Range
    {
        T first;
        T last;
    };

    struct RangeLess
    {
        bool operator()(const Range& left, const Range& right) const
        {
            return left.first < right.first;
        }
    };

    std::set<Range, RangeLess> freeRanges;
public:
    IdPool(T firstId = std::numeric_limits<T>::min(), T lastId = std::numeric_limits<T>::max())
    {
        if (lastId < firstId)
        {
            std::swap(firstId, lastId);
        }

        freeRanges.insert(Range{ firstId, lastId });
    }

    std::optional<T> acquireId()
    {
        if (freeRanges.empty())
        {
            return std::nullopt;
        }

        auto current = freeRanges.begin();
        T id = current->first;

        if (current->first == current->last)
        {
            freeRanges.erase(current);
        }
        else
        {
            Range range = *current;
            freeRanges.erase(current);
            range.first = static_cast<T>(range.first + 1);
            freeRanges.insert(range);
        }

        return id;
    }

    void releaseId(T id)
    {
        auto next = freeRanges.lower_bound(Range{ id, id });

        if (next != freeRanges.begin())
        {
            auto previous = std::prev(next);

            if (previous->first <= id && id <= previous->last)
            {
                return;
            }

            if (previous->last != std::numeric_limits<T>::max() && previous->last + 1 == id)
            {
                id = previous->first;
                freeRanges.erase(previous);
            }
        }

        if (next != freeRanges.end())
        {
            if (id != std::numeric_limits<T>::max() && id + 1 == next->first)
            {
                Range merged{ id, next->last };
                freeRanges.erase(next);
                freeRanges.insert(merged);
                return;
            }
        }

        freeRanges.insert(Range{ id, id });
    }
};