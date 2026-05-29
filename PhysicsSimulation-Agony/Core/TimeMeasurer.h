#pragma once
#include <chrono>

class TimeMeasurer
{
    std::chrono::steady_clock::time_point start;
public:
    TimeMeasurer() : start(std::chrono::steady_clock::now()) {}

    void reset();

    double elapsed_seconds() const;

    long long elapsed_milliseconds() const;
};