#include "experiment_metrics.h"

#include <algorithm>
#include <fstream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <time.h>
#include <unistd.h>

ProcessCpuMeter::ProcessCpuMeter()
{
    ticks_per_second_ = ::sysconf(_SC_CLK_TCK);
    cpu_count_ = ::sysconf(_SC_NPROCESSORS_ONLN);
    if (ticks_per_second_ <= 0)
        ticks_per_second_ = 100;
    if (cpu_count_ <= 0)
        cpu_count_ = 1;
}

std::uint64_t ProcessCpuMeter::process_ticks()
{
    std::ifstream stat("/proc/self/stat");
    std::string line;
    std::getline(stat, line);
    const auto close_paren = line.rfind(')');
    if (!stat || close_paren == std::string::npos)
        throw std::runtime_error("cannot read /proc/self/stat");

    std::istringstream fields(line.substr(close_paren + 2));
    std::string value;
    std::uint64_t user_ticks = 0;
    std::uint64_t system_ticks = 0;
    /* The substring starts at field 3; utime/stime are fields 14 and 15. */
    for (int field = 3; field <= 15; ++field) {
        if (!(fields >> value))
            throw std::runtime_error("truncated /proc/self/stat");
        if (field == 14)
            user_ticks = std::stoull(value);
        else if (field == 15)
            system_ticks = std::stoull(value);
    }
    return user_ticks + system_ticks;
}

void ProcessCpuMeter::start()
{
    start_ticks_ = process_ticks();
    start_time_ = std::chrono::steady_clock::now();
}

CpuUsage ProcessCpuMeter::stop() const
{
    const auto end_ticks = process_ticks();
    const double wall_seconds = std::chrono::duration<double>(
                                    std::chrono::steady_clock::now() - start_time_).count();
    const double cpu_seconds = static_cast<double>(end_ticks - start_ticks_) /
                               static_cast<double>(ticks_per_second_);
    CpuUsage result;
    if (wall_seconds > 0.0)
        result.process_percent = cpu_seconds / wall_seconds * 100.0;
    result.machine_percent = result.process_percent / static_cast<double>(cpu_count_);
    return result;
}

double mean(const std::vector<double>& values)
{
    if (values.empty())
        return 0.0;
    return std::accumulate(values.begin(), values.end(), 0.0) /
           static_cast<double>(values.size());
}

double percentile_ms(std::vector<double> values, double percentile)
{
    if (values.empty())
        return 0.0;
    std::sort(values.begin(), values.end());
    const double position = (values.size() - 1) * percentile;
    const auto lower = static_cast<std::size_t>(position);
    const auto upper = std::min(lower + 1, values.size() - 1);
    const double fraction = position - lower;
    return values[lower] * (1.0 - fraction) + values[upper] * fraction;
}

double current_thread_cpu_ms()
{
    struct timespec value {};
    if (::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0)
        throw std::runtime_error("cannot read thread CPU clock");
    return static_cast<double>(value.tv_sec) * 1000.0 +
           static_cast<double>(value.tv_nsec) / 1.0e6;
}
