// ============================================================
// perf_config.h —— 统一运行配置（T00）
// ============================================================
// 目的：所有正式入口使用同一配置对象；参数来源可审计（default / env）；
// 非法值显式报错（禁止 atoi 静默变成另一种有效配置）。
// 加载时机：main() 早期调用 p3_config_load_from_env()；引擎侧懒加载兜底。
// 打印：p3_config_print() 启动打印全部生效值；p3_config_json() 供 run_config.json。
#ifndef P3_PERF_CONFIG_H
#define P3_PERF_CONFIG_H

#include <cstdint>
#include <string>

struct P3RuntimeConfig {
    // ---- 调度频率（正式档：light=2, sign=20, object=1） ----
    int light_interval = 2;
    int sign_interval = 20;
    int object_interval = 1;
    int lane_interval = 20;      // UFLD 车道（默认关闭时仅记录）

    // ---- 灯真伪复核 ----
    bool light_verify = true;
    int light_verify_topk = 2;
    float light_verify_conf = 0.50f;
    // [ARROW-RECALL] Locator confidence for the light detector. Default
    // 0.35 keeps the production value; lower values admit smaller or
    // farther lamps (the verify filter and ATLAS guard still gate them).
    float light_conf = 0.35f;

    // ---- 跟踪 ----
    bool light_track = true;
    int light_track_hold = 2;
    float object_ema = 0.0f;     // EMA 默认 0 = 新检测直接替换
    int object_track_hold = 2;
    bool object_track_flow = true;

    // ---- 功能开关 ----
    bool enable_ufld = false;
    bool enable_speed = true;    // 注意：main 历史默认 true / HDMI 脚本默认 0（显式报告）

    // ---- 证据时效（毫秒；T02-B 使用；台架初始工程值，非安全认证标准） ----
    // [LIGHT-TTL 2026-10-22] 300 -> 500 ms，按源时间口径推导：灯推理隔帧
    // 刷新（7 FPS 下约 290 ms）+ 决策点距采集的延迟（约 270 ms，含取帧
    // 180 ms 与处理 90 ms）≈ 410 ms 最坏值，500 ms 留约 90 ms 余量。
    // 旧的 300 ms 是消费时刻口径的遗留值；改为源时间后隔帧那一步必然
    // 过期，导致稳定绿灯被强制降 UNKNOWN 的闪烁。过期仍降级，红灯仍立即。
    int light_ttl_ms = 500;
    // [USER-A 2026-10-22] 对象源窗口 250 -> 400 ms（与 Twin 源窗口一致）。
    int object_ttl_ms = 400;
    int sign_ttl_ms = 1500;
    int lane_stale_ms = 250;
    int red_memory_ms = 1500;
    int alarm_hold_ms = 1000;

    // ---- 审计 ----
    // 每项最终来源："default" 或缺省值；"env:P3_XXX=raw" 表示环境变量覆盖。
    std::string source_light_interval;
    std::string source_sign_interval;
    std::string source_object_interval;
    std::string source_lane_interval;
    std::string source_light_verify;
    std::string source_light_verify_topk;
    std::string source_light_verify_conf;
    std::string source_light_conf;
    std::string source_enable_speed;
    std::string source_enable_ufld;
    bool enable_sign = true;  // default launcher explicitly disables optional sign model
    std::string source_enable_sign;
};

// 进程级单例（首次调用时自动从环境变量加载；非法值 fail-fast 并给出原因）。
P3RuntimeConfig& p3_config();

// 显式加载（main 早期调用）。env 非法时返回 false 并写 error。
bool p3_config_load_from_env(P3RuntimeConfig* out, std::string* error);

// 启动打印（单行/多行摘要，含每项来源）。
void p3_config_print(const P3RuntimeConfig& cfg);

// 机读 JSON（供 run_config.json 落盘）。run_id/argv_json/ai_mode 等运行时
// 信息由调用方（main）附加；缺省时省略对应字段。
std::string p3_config_json(const P3RuntimeConfig& cfg,
                           const std::string& run_id = std::string(),
                           const std::string& argv_json = std::string(),
                           int ai_mode = -1, bool fpga_decision = false,
                           int prefetch = -1);

#endif  // P3_PERF_CONFIG_H
