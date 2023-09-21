#ifndef STOP_WATCH_H
#define STOP_WATCH_H

#include <chrono>

// Return lap time [msec] during the previous lap()
struct StopWatch {
    StopWatch() { pre_ = std::chrono::high_resolution_clock::now(); }

    double lap() {
        auto tmp = std::chrono::high_resolution_clock::now();
        auto dur = tmp - pre_;
        pre_ = tmp;
        return std::chrono::duration_cast<std::chrono::nanoseconds>(dur).count() / 1000000.0;
    }
    std::chrono::high_resolution_clock::time_point pre_;
};

#endif