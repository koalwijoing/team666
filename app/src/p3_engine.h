// ============================================================
// p3_engine.h —— 感知引擎接入层（P3-a 集成用）
// ============================================================
// 用途：把「厂商 app_pcie 实时收图」与「我们已验收的感知引擎」接起来。
//
// 集成方式（只改一行）：
//   厂商 main.cpp 里  yolo_run_main(img_display);
//   换成            p3_engine_run(img_display);     ← 同签名 cv::Mat&
//   其余（DMA 拉帧 / RGA 色转 / 显示）全部不动。
//
// 输入几何（2026-09-18 板上实测裁定）：
//   实时帧 = **1280×360 SBS 立体对**：[左目 640×360 | 右目 640×360]，RGB888（RGA 已转）
//   本层内置：取左半 640×360 → letterbox 到 640×640 → 推理
//
// 依赖：librknnrt.so（板载）+ rknn_api.h（app_pcie/includes 有）+ postprocess.h/.c（我方已交付）
// 模型：/root/board_deploy/{lisa_v4_headless.rknn, sign_det_v3_headless.rknn}
// ============================================================
#ifndef P3_ENGINE_H
#define P3_ENGINE_H

#include "p3_light_verify.h"

#include <cstdint>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

struct P3Det {
    float x1, y1, x2, y2;   // 左目 640×360 坐标系
    float score;
    int   cls;
    int   model_id;         // 0=灯(lisa) 1=牌(sign_det) 2=行人/车辆(COCO)
    std::string label;
    // [T02-A] 灯复核状态（kLightVerify* 常量，见 p3_light_verify.h）；
    // 非灯检测保持 NotEvaluated 默认值。
    int verify_state = 0;
    // [N1-5/T02-B] 观察元数据：该检测所属推理观察的身份与时间。
    //   跟踪（光流）只更新坐标，不得刷新这些字段；缓存复用时把 origin 标 2。
    std::uint64_t obs_run_id = 0;      // 生成该检测的成功推理观察 id
    double obs_capture_ms = 0;         // 该观察输入的就绪时刻（dequeue 近似）
    double obs_infer_done_ms = 0;      // 推理完成时刻
    int obs_origin = 0;                // 0=inferred 1=tracked 2=cached
    int obs_age_frames = 0;            // frames since last model detection (0=now)
};

// [N1-5] 灯观察快照：供决策桥区分"本帧成功新观察 / 失败 / 未跑"，
// 并据此做防抖去重、证据年龄与 TTL 判定。
struct P3LampObservation {
    std::uint64_t run_id = 0;       // 最近成功灯推理的观察 id（0=从未成功）
    double capture_done_ms = 0;     // 最近一帧输入的采集就绪时刻
    double infer_done_ms = 0;       // 最近一次成功灯推理完成时刻
    int status = 0;                 // 0=未跑/缓存 1=成功 2=失败
};

// [C-R2] 对象观察快照：整次观察（含成功 0 框）的 run 身份与源时间。
struct P3ObjectObservation {
    std::uint64_t run_id = 0;       // 最近成功对象推理的观察 id（0=从未成功）
    double capture_done_ms = 0;     // 该成功观察的输入采集就绪时刻
    int status = 0;                 // 0=未跑/缓存 1=成功 2=失败
};

class P3Engine {
public:
    // 初始化：加载灯、牌和 COCO 行人/车辆三个剥头模型。
    bool init(const std::string& light_rknn, const std::string& sign_rknn,
              const std::string& object_rknn,
              const std::string& light_filter_rknn = "");

    // 单帧推理 + 画框（原地修改 img，img 为 1280×360 SBS RGB888）
    // sign_interval: 牌模型降频（默认 20 → 约 1fps @20FPS 输出）。
    //   理由：牌是静态目标，我方架构本就是"灯每帧 + 牌低频"；
    //   实测双模型同帧跑 82ms→10.4FPS，降频后预期 ~45ms→20+FPS。
    //   降频期间沿用上次牌结果（persist），避免框闪烁。
    // [T00] 默认参数与 perf_config.h 统一配置默认一致：object_interval 历史上
    // 声明为 5、实际生效 1，已统一为 1（正式入口 p3_engine_run 走配置对象）。
    std::vector<P3Det> run(cv::Mat& img, float light_conf = 0.45f,
                           float sign_conf = 0.35f, float object_conf = 0.25f,
                           int sign_interval = 20, int object_interval = 1,
                           int light_interval = 2);

    // [T01] 预热四个模型（含复核器固定合法 crop 预热）；预热后应调用
    // reset_state()，不得让预热结果污染正式首帧。
    void warmup_models();
    // [T01] 清空跟踪/持久化/审计计数（保留模型与配置）。
    void reset_state();

    // 计时（ms）
    double last_pre_ms = 0, last_light_ms = 0, last_sign_ms = 0;
    double last_object_ms = 0, last_pp_ms = 0, last_total_ms = 0;
    // [N1-1] 每帧模型状态（每帧开始清零，禁止沿用上帧结果伪装本帧工作量）：
    //   scheduled=调度决定运行；attempted=实际进入了推理调用；
    //   succeeded=inputs_set/run/outputs_get 全部成功；failed=attempted 但失败。
    //   模型耗时为"本帧成功推理"才有值；未跑/失败帧为 0。
    bool last_light_scheduled = false, last_light_attempted = false,
         last_light_succeeded = false, last_light_failed = false;
    bool last_sign_scheduled = false, last_sign_attempted = false,
         last_sign_succeeded = false, last_sign_failed = false;
    bool last_object_scheduled = false, last_object_attempted = false,
         last_object_succeeded = false, last_object_failed = false;
    // [T02-A] 复核降级状态：verify 开启但复核执行出错/分类器未就绪时置位；
    // 正式完整功能档禁止把该状态当作"复核开启且有效"。
    bool light_verify_degraded = false;
    // [T02-A] 最近一帧各灯候选的复核状态计数（审计用）。
    int last_verify_accepted = 0, last_verify_rejected = 0,
        last_verify_not_evaluated = 0, last_verify_error = 0;
    // [T01] 结果年龄：本帧输入采集完成时刻（steady_clock 绝对毫秒，由
    // p3_engine_note_capture 设置）。当前口径为"取帧返回时刻"，见实现注释。
    double last_capture_done_ms = 0;
    // [N1-5] 每模型"成功推理观察"计数（每次成功推理 +1；用于决策桥按不同
    // 观察去重投票，缓存/光流帧不增加）。
    std::uint64_t light_run_id = 0, sign_run_id = 0, object_run_id = 0;
    // [N1-5] 最近一次成功灯推理完成时刻（毫秒）。
    double last_light_infer_done_ms = 0;
    // [C-R2] 最近一次成功对象推理的观察身份与采集返回时刻（含 0 框成功）。
    std::uint64_t last_object_success_run_id = 0;
    double last_object_success_capture_ms = 0.0;

    void release();

    // [T02-D] 桌面失败注入测试访问点（tests/test_engine_rknn_guard.cpp）。
    friend struct P3EngineTestAccess;

private:
    int frame_idx_ = 0;              // 帧计数（驱动牌模型降频）
    std::vector<P3Det> last_sign_;   // 上次牌检出（persist 用）
    int sign_age_ = 0;               // 距上次牌检出的帧数（超龄则丢弃）
    std::vector<P3Det> last_object_; // 上次行人/车辆检出（persist 用）
    int object_age_ = 0;             // 距上次目标检出的帧数
    std::vector<P3Det> last_light_;  // 上次灯检出（降频 persist 用）
    int light_age_ = 0;              // 距上次灯检出的帧数
    struct ModelCtx {
        void*  ctx = nullptr;
        int    nc = 7, imgsz = 640;
        int    n_out = 0;
        int    chs[8]{}, hs[8]{}, ws[8]{};
        size_t sizes[8]{};
        int    in_size = 0;            // 输入缓冲字节数 (1,3,640,640) NHWC uint8
        // [PERF-INT8 2026-10-22] Per-output quantization metadata, queried once
        // at load. The float path (want_float=1) ignores these; the int8 path
        // dequantizes with exactly the same affine formula the driver uses.
        int    out_type[8]{};          // rknn_tensor_type
        int    out_qnt[8]{};           // rknn_tensor_qnt_type
        float  out_scale[8]{};
        int    out_zp[8]{};
        unsigned int out_elems[8]{};
        unsigned int out_bytes[8]{};   // raw tensor bytes (attr.size)
        bool   int8_ok = false;        // all outputs affine int8, scale > 0
        bool   ready = false;
    };
    ModelCtx light_, sign_, object_, light_filter_;
    bool load_model(ModelCtx& m, const std::string& path, int nc, int imgsz);
    // [T02-A] 四态复核：NotEvaluated(开关关闭) / Accepted / Rejected / Error。
    // 不再用 bool true 表示"推理出错"或"模型未加载"。
    LightVerifyResult light_is_real(const cv::Mat& sbs, const P3Det& detection);
    struct ObjectTrack {
        P3Det det;
        int missed = 0;
        float vx = 0.0f;
        float vy = 0.0f;
    };
    void update_object_tracks(const std::vector<P3Det>& detections);
    void age_object_tracks();
    std::vector<ObjectTrack> object_tracks_;
    std::vector<ObjectTrack> light_tracks_;
    cv::Mat prev_sign_gray_;
    cv::Mat prev_object_gray_;
    cv::Mat prev_light_gray_;
    void track_sign_boxes(const cv::Mat& current_gray);
    void track_object_boxes(const cv::Mat& current_gray);
    void update_light_tracks(const std::vector<P3Det>& detections);
    void age_light_tracks();
    void track_light_boxes(const cv::Mat& current_gray);
    static cv::Mat sbs_left_gray(const cv::Mat& sbs);
    // SBS 左目裁切 + letterbox（等比缩放 + 灰边），返回缩放比与偏移
    static cv::Mat sbs_left_letterbox(const cv::Mat& sbs, int imgsz,
                                      float& ratio, int& dw, int& dh);
    // 完整 SBS 帧 letterbox：牌子模型对远近尺度和双目上下文更敏感。
    static cv::Mat sbs_full_letterbox(const cv::Mat& sbs, int imgsz,
                                      float& ratio, int& dw, int& dh);
};

// 与厂商 yolo_run_main 同签名的便捷入口（集成只改这一行）
void p3_engine_run(cv::Mat& img_display);
const std::vector<P3Det>& p3_engine_last_detections();
// [T01] 主循环在拿到输入帧后调用（steady_clock 绝对毫秒），用于结果年龄。
void p3_engine_note_capture(double steady_ms);
// [N1-1] 本帧各模型的状态（供逐帧 CSV：scheduled/attempted/succeeded/failed；
// cached 由 !scheduled 推断）。
struct P3ModelFrameState {
    bool scheduled = false;
    bool attempted = false;
    bool succeeded = false;
    bool failed = false;
};
P3ModelFrameState p3_engine_last_light_state();
P3ModelFrameState p3_engine_last_sign_state();
P3ModelFrameState p3_engine_last_object_state();
// [N1-5] 灯观察快照（供决策桥做观察去重与证据 TTL 判定）。
P3LampObservation p3_engine_last_light_observation();
// [C-R2] 对象观察快照（含成功 0 框；供顾问做观察级源年龄判定）。
P3ObjectObservation p3_engine_last_object_observation();
// [T01] 四模型预热与状态重置（正式主循环开始前调用）。
void p3_engine_warmup();
void p3_engine_reset_state();

#endif // P3_ENGINE_H
