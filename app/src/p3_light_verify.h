// ============================================================
// p3_light_verify.h —— 灯真伪复核四态与结果组装（T02-A）
// ============================================================
// 设计要点（任务书 T02-A）：
//   - 复核返回四态：Accepted / Rejected / NotEvaluated / Error，
//     不用 bool true 表示"推理出错"。
//   - verify_topk 是预算，不是"其余灯不存在"：超预算候选保留为
//     未复核证据（NotEvaluated），不删除其方向红灯。
//   - Rejected 不进入输出（不参与跟踪建轨、不参与决策投票）。
//   - Error / NotEvaluated 保留在输出供决策层保守处理，并置 degraded。
//   - 每个候选最多出现一次（修复"未复核候选无条件进入输出 + 通过
//     复核后重复加入"的缺陷）。
// 组装逻辑为纯函数，供桌面测试直接覆盖（不依赖 RKNN）。
#ifndef P3_LIGHT_VERIFY_H
#define P3_LIGHT_VERIFY_H

#include <cstddef>
#include <vector>

enum class LightVerifyResult {
    NotEvaluated = 0,  // 未复核（预算外 / 复核开关关闭）
    Accepted = 1,      // 复核通过（判定为真灯）
    Rejected = 2,      // 复核拒绝（判定为非灯）
    Error = 3,         // 复核执行出错（模型未加载 / 推理失败）
};

// 与 P3Det::verify_state 对应的 int 常量。
constexpr int kLightVerifyNotEvaluated = 0;
constexpr int kLightVerifyAccepted = 1;
constexpr int kLightVerifyRejected = 2;
constexpr int kLightVerifyError = 3;

// 组装复核后的灯输出列表。模板化以接受任意带 verify_state 字段的检测类型
// （生产类型 P3Det；测试也可用轻量类型）。
//   ranked          按 score 降序排好的原始候选
//   verify_limit    topk 预算（前 verify_limit 个候选对应 verify_results）
//   verify_results  与 ranked 前缀对应的复核结果（长度可短于 limit）
//   degraded        任一候选复核 Error 时置 true
template <typename DetT>
std::vector<DetT> assemble_light_output(
    const std::vector<DetT>& ranked, std::size_t verify_limit,
    const std::vector<LightVerifyResult>& verify_results, bool* degraded)
{
    std::vector<DetT> out;
    out.reserve(ranked.size());
    bool deg = false;
    for (std::size_t i = 0; i < ranked.size(); ++i) {
        DetT det = ranked[i];
        LightVerifyResult result = LightVerifyResult::NotEvaluated;
        if (i < verify_limit && i < verify_results.size())
            result = verify_results[i];
        switch (result) {
        case LightVerifyResult::Accepted:
            det.verify_state = kLightVerifyAccepted;
            out.push_back(det);
            break;
        case LightVerifyResult::Rejected:
            // 拒绝候选不进入输出（不建轨、不投票）。
            break;
        case LightVerifyResult::Error:
            det.verify_state = kLightVerifyError;
            out.push_back(det);
            deg = true;
            break;
        case LightVerifyResult::NotEvaluated:
        default:
            det.verify_state = kLightVerifyNotEvaluated;
            out.push_back(det);
            break;
        }
    }
    if (degraded)
        *degraded = deg;
    return out;
}

#endif  // P3_LIGHT_VERIFY_H
