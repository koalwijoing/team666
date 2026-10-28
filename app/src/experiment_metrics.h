#ifndef EXPERIMENT_METRICS_H
#define EXPERIMENT_METRICS_H

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

struct CpuUsage {
    double process_percent = 0.0;       // 100% means one fully occupied core.
    double machine_percent = 0.0;       // Normalized by online CPU count.
};

class ProcessCpuMeter {
public:
    ProcessCpuMeter();
    void start();
    CpuUsage stop() const;

private:
    static std::uint64_t process_ticks();
    std::uint64_t start_ticks_ = 0;
    std::chrono::steady_clock::time_point start_time_;
    long ticks_per_second_ = 100;
    long cpu_count_ = 1;
};

double percentile_ms(std::vector<double> values, double percentile);
double mean(const std::vector<double>& values);
double current_thread_cpu_ms();

#endif
