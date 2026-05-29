#include "TimeMeasurer.h"

void TimeMeasurer::reset()
{
    start = std::chrono::steady_clock::now();
}

double TimeMeasurer::elapsed_seconds() const
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - start).count();
}

long long TimeMeasurer::elapsed_milliseconds() const
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}
