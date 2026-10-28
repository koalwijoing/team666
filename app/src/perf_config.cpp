// ============================================================
// perf_config.cpp —— 统一运行配置实现（T00）
// ============================================================
#include "perf_config.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>

namespace {

// 解析整数：env 未设置 → 默认；设置但非法/越界 → 报错（不静默容错）。
bool parse_int_env(const char* key, int fallback, int lo, int hi,
                   int* out, std::string* source, std::string* error)
{
    const char* raw = std::getenv(key);
    if (raw == nullptr || *raw == '\0') {
        *out = fallback;
        *source = "default";
        return true;
    }
    char* end = nullptr;
    const long value = std::strtol(raw, &end, 10);
    if (end == raw || *end != '\0') {
        if (error)
            *error = std::string(key) + "='" + raw + "' is not an integer";
        return false;
    }
    if (value < lo || value > hi) {
        if (error) {
            std::ostringstream os;
            os << key << "=" << value << " out of range [" << lo << "," << hi << "]";
            *error = os.str();
        }
        return false;
    }
    *out = static_cast<int>(value);
    *source = std::string("env:") + key + "=" + raw;
    return true;
}

bool parse_float_env(const char* key, float fallback, float lo, float hi,
                     float* out, std::string* source, std::string* error)
{
    const char* raw = std::getenv(key);
    if (raw == nullptr || *raw == '\0') {
        *out = fallback;
        *source = "default";
        return true;
    }
    char* end = nullptr;
    const double value = std::strtod(raw, &end);
    if (end == raw || *end != '\0') {
        if (error)
            *error = std::string(key) + "='" + raw + "' is not a number";
        return false;
    }
    if (value < lo || value > hi) {
        if (error) {
            std::ostringstream os;
            os << key << "=" << value << " out of range [" << lo << "," << hi << "]";
            *error = os.str();
        }
        return false;
    }
    *out = static_cast<float>(value);
    *source = std::string("env:") + key + "=" + raw;
    return true;
}

// 开关：接受 0/1/f/false/true（首字符判定与 main 历史 env_flag 一致）。
bool parse_flag_env(const char* key, bool fallback, bool* out,
                    std::string* source, std::string* error)
{
    const char* raw = std::getenv(key);
    if (raw == nullptr || *raw == '\0') {
        *out = fallback;
        *source = "default";
        return true;
    }
    const char c = raw[0];
    if (c == '0' || c == 'f' || c == 'F')
        *out = false;
    else if (c == '1' || c == 't' || c == 'T')
        *out = true;
    else {
        if (error)
            *error = std::string(key) + "='" + raw + "' is not a flag (0/1)";
        return false;
    }
    *source = std::string("env:") + key + "=" + raw;
    return true;
}

}  // namespace

bool p3_config_load_from_env(P3RuntimeConfig* out, std::string* error)
{
    P3RuntimeConfig cfg;
    if (!parse_flag_env("P3_ENABLE_SIGN", true, &cfg.enable_sign,
                        &cfg.source_enable_sign, error))
        return false;
    if (!parse_int_env("P3_LIGHT_INTERVAL", 2, 1, 100, &cfg.light_interval,
                       &cfg.source_light_interval, error))
        return false;
    if (!parse_int_env("P3_SIGN_INTERVAL", 20, 1, 1000, &cfg.sign_interval,
                       &cfg.source_sign_interval, error))
        return false;
    if (!parse_int_env("P3_OBJECT_INTERVAL", 1, 1, 100, &cfg.object_interval,
                       &cfg.source_object_interval, error))
        return false;
    if (!parse_int_env("P3_LANE_INTERVAL", 20, 1, 1000, &cfg.lane_interval,
                       &cfg.source_lane_interval, error))
        return false;
    if (!parse_flag_env("P3_LIGHT_VERIFY", true, &cfg.light_verify,
                        &cfg.source_light_verify, error))
        return false;
    if (!parse_int_env("P3_LIGHT_VERIFY_TOPK", 2, 1, 64,
                       &cfg.light_verify_topk, &cfg.source_light_verify_topk,
                       error))
        return false;
    if (!parse_float_env("P3_LIGHT_VERIFY_CONF", 0.50f, 0.05f, 0.99f,
                         &cfg.light_verify_conf, &cfg.source_light_verify_conf,
                         error))
        return false;
    if (!parse_float_env("P3_LIGHT_CONF", 0.35f, 0.05f, 0.99f,
                         &cfg.light_conf, &cfg.source_light_conf, error))
        return false;
    if (!parse_flag_env("P3_ENABLE_SPEED", true, &cfg.enable_speed,
                        &cfg.source_enable_speed, error))
        return false;
    if (!parse_flag_env("P3_ENABLE_UFLD", false, &cfg.enable_ufld,
                        &cfg.source_enable_ufld, error))
        return false;

    // 其余开关/保持帧数：与历史语义一致，非法值报错。
    std::string dummy;
    if (!parse_flag_env("P3_LIGHT_TRACK", true, &cfg.light_track, &dummy, error))
        return false;
    if (!parse_int_env("P3_LIGHT_TRACK_HOLD", 2, 1, 100,
                       &cfg.light_track_hold, &dummy, error))
        return false;
    if (!parse_float_env("P3_OBJECT_EMA", 0.0f, 0.0f, 0.95f, &cfg.object_ema,
                         &dummy, error))
        return false;
    if (!parse_int_env("P3_OBJECT_TRACK_HOLD", 2, 1, 100,
                       &cfg.object_track_hold, &dummy, error))
        return false;

    *out = cfg;
    return true;
}

P3RuntimeConfig& p3_config()
{
    static P3RuntimeConfig cfg;
    static bool loaded = false;
    if (!loaded) {
        std::string error;
        if (!p3_config_load_from_env(&cfg, &error)) {
            std::fprintf(stderr,
                         "[p3-config] FATAL: invalid configuration: %s\n",
                         error.c_str());
            std::exit(2);  // fail-fast：非法配置不允许静默变成另一种有效配置
        }
        loaded = true;
    }
    return cfg;
}

void p3_config_print(const P3RuntimeConfig& cfg)
{
    std::printf("[p3-config] enable_sign=%d(%s)\n",cfg.enable_sign ? 1 : 0,
                cfg.source_enable_sign.c_str());
    std::printf(
        "[p3-config] light_interval=%d(%s) sign_interval=%d(%s) "
        "object_interval=%d(%s) lane_interval=%d(%s)\n",
        cfg.light_interval, cfg.source_light_interval.c_str(),
        cfg.sign_interval, cfg.source_sign_interval.c_str(),
        cfg.object_interval, cfg.source_object_interval.c_str(),
        cfg.lane_interval, cfg.source_lane_interval.c_str());
    std::printf(
        "[p3-config] light_verify=%d(%s) verify_topk=%d(%s) verify_conf=%.2f(%s) "
        "light_conf=%.2f(%s)\n",
        cfg.light_verify ? 1 : 0, cfg.source_light_verify.c_str(),
        cfg.light_verify_topk, cfg.source_light_verify_topk.c_str(),
        cfg.light_verify_conf, cfg.source_light_verify_conf.c_str(),
        cfg.light_conf, cfg.source_light_conf.c_str());
    std::printf(
        "[p3-config] light_track=%d hold=%d | object_ema=%.2f track_hold=%d | "
        "enable_ufld=%d(%s) enable_speed=%d(%s)\n",
        cfg.light_track ? 1 : 0, cfg.light_track_hold, cfg.object_ema,
        cfg.object_track_hold, cfg.enable_ufld ? 1 : 0,
        cfg.source_enable_ufld.c_str(), cfg.enable_speed ? 1 : 0,
        cfg.source_enable_speed.c_str());
    std::printf(
        "[p3-config] evidence_ttl_ms: light=%d object=%d sign=%d lane_stale=%d "
        "red_memory=%d alarm_hold=%d\n",
        cfg.light_ttl_ms, cfg.object_ttl_ms, cfg.sign_ttl_ms, cfg.lane_stale_ms,
        cfg.red_memory_ms, cfg.alarm_hold_ms);
}

std::string p3_config_json(const P3RuntimeConfig& cfg,
                           const std::string& run_id,
                           const std::string& argv_json, int ai_mode,
                           bool fpga_decision, int prefetch)
{
    std::ostringstream os;
    os << "{\n"
       << "  \"run_id\": \"" << run_id << "\",\n"
       << "  \"ai_mode\": " << ai_mode << ",\n"
       << "  \"fpga_decision\": " << (fpga_decision ? 1 : 0) << ",\n"
       << "  \"prefetch\": " << prefetch << ",\n"
       << "  \"argv\": " << (argv_json.empty() ? "[]" : argv_json) << ",\n"
       << "  \"light_interval\": " << cfg.light_interval << ",\n"
       << "  \"sign_interval\": " << cfg.sign_interval << ",\n"
       << "  \"enable_sign\": " << (cfg.enable_sign ? 1 : 0) << ",\n"
       << "  \"object_interval\": " << cfg.object_interval << ",\n"
       << "  \"lane_interval\": " << cfg.lane_interval << ",\n"
       << "  \"light_verify\": " << (cfg.light_verify ? 1 : 0) << ",\n"
       << "  \"light_verify_topk\": " << cfg.light_verify_topk << ",\n"
       << "  \"light_verify_conf\": " << cfg.light_verify_conf << ",\n"
       << "  \"light_conf\": " << cfg.light_conf << ",\n"
       << "  \"light_track\": " << (cfg.light_track ? 1 : 0) << ",\n"
       << "  \"light_track_hold\": " << cfg.light_track_hold << ",\n"
       << "  \"object_ema\": " << cfg.object_ema << ",\n"
       << "  \"object_track_hold\": " << cfg.object_track_hold << ",\n"
       << "  \"enable_ufld\": " << (cfg.enable_ufld ? 1 : 0) << ",\n"
       << "  \"enable_speed\": " << (cfg.enable_speed ? 1 : 0) << ",\n"
       << "  \"light_ttl_ms\": " << cfg.light_ttl_ms << ",\n"
       << "  \"object_ttl_ms\": " << cfg.object_ttl_ms << ",\n"
       << "  \"sign_ttl_ms\": " << cfg.sign_ttl_ms << ",\n"
       << "  \"lane_stale_ms\": " << cfg.lane_stale_ms << ",\n"
       << "  \"red_memory_ms\": " << cfg.red_memory_ms << ",\n"
       << "  \"alarm_hold_ms\": " << cfg.alarm_hold_ms << ",\n"
       << "  \"sources\": {\n"
       << "    \"light_interval\": \"" << cfg.source_light_interval << "\",\n"
       << "    \"sign_interval\": \"" << cfg.source_sign_interval << "\",\n"
       << "    \"enable_sign\": \"" << cfg.source_enable_sign << "\",\n"
       << "    \"object_interval\": \"" << cfg.source_object_interval << "\",\n"
       << "    \"light_verify\": \"" << cfg.source_light_verify << "\",\n"
       << "    \"enable_speed\": \"" << cfg.source_enable_speed << "\"\n"
       << "  }\n"
       << "}\n";
    return os.str();
}
