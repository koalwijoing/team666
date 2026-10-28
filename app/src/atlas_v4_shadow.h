// ATLAS v4 attributes, optionally constrained by simulation agreement guard.
// No direct BAR writes or real actuator API.
#ifndef ATLAS_V4_SHADOW_H
#define ATLAS_V4_SHADOW_H
#include <opencv2/opencv.hpp>
#include "rknn_api.h"
#include "atlas_v4_decision_guard.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

inline cv::Mat atlas_v4_prepare(const cv::Mat& original) {
    if (original.empty() || original.type() != CV_8UC3)
        throw std::runtime_error("ATLAS v4 requires nonempty BGR image");
    cv::Mat image = original;
    if (std::min(original.cols, original.rows) >= 4) {
        cv::Mat hsv, mask(original.rows, original.cols, CV_8U, cv::Scalar(0));
        cv::cvtColor(original, hsv, cv::COLOR_BGR2HSV);
        for (int y=0; y<hsv.rows; ++y) for (int x=0; x<hsv.cols; ++x) {
            const auto p=hsv.at<cv::Vec3b>(y,x);
            if ((p[0]<=42 || p[0]>=168 || (p[0]>=45 && p[0]<=95)) &&
                p[1]>=80 && p[2]>=80) mask.at<unsigned char>(y,x)=1;
        }
        cv::Mat labels, stats, centroids;
        const int n=cv::connectedComponentsWithStats(mask,labels,stats,centroids,8);
        struct Component { double brightness; cv::Rect rect; };
        std::vector<Component> components;
        for (int i=1; i<n; ++i) {
            const int w=stats.at<int>(i,cv::CC_STAT_WIDTH), h=stats.at<int>(i,cv::CC_STAT_HEIGHT);
            if (stats.at<int>(i,cv::CC_STAT_AREA)<3 || w<2 || h<2) continue;
            cv::Rect r(stats.at<int>(i,cv::CC_STAT_LEFT),stats.at<int>(i,cv::CC_STAT_TOP),w,h);
            double brightness=0;
            for (int y=r.y; y<r.y+r.height; ++y) for (int x=r.x; x<r.x+r.width; ++x)
                if (labels.at<int>(y,x)==i) brightness+=hsv.at<cv::Vec3b>(y,x)[2];
            components.push_back({brightness,r});
        }
        std::sort(components.begin(),components.end(),[](const Component& a,const Component& b) {
            return a.brightness>b.brightness;
        });
        if (!components.empty() && (components.size()==1 ||
            components[1].brightness<=components[0].brightness*.65)) {
            const auto r=components[0].rect;
            const int m=std::max(2,int(std::ceil(std::max(r.width,r.height)*.3)));
            const int x1=std::max(0,r.x-m), y1=std::max(0,r.y-m);
            const int x2=std::min(original.cols,r.x+r.width+m), y2=std::min(original.rows,r.y+r.height+m);
            image=original(cv::Rect(x1,y1,x2-x1,y2-y1));
        }
    }
    const double ratio=128.0/std::max(image.cols,image.rows);
    const int w=std::max(1,int(std::nearbyint(image.cols*ratio)));
    const int h=std::max(1,int(std::nearbyint(image.rows*ratio)));
    cv::Mat result(128,128,CV_8UC3,cv::Scalar(0)), resized;
    cv::resize(image,resized,cv::Size(w,h),0,0,cv::INTER_LINEAR);
    resized.copyTo(result(cv::Rect((128-w)/2,(128-h)/2,w,h)));
    return result;
}

inline std::vector<float> atlas_v4_prob(const float* p, int n) {
    if (!p) throw std::runtime_error("ATLAS v4 null logits");
    for (int i=0;i<n;++i) if (!std::isfinite(p[i]))
        throw std::runtime_error("ATLAS v4 nonfinite logits");
    const float maximum=*std::max_element(p,p+n);
    std::vector<float> probabilities(n); float total=0;
    for (int i=0;i<n;++i) total+=(probabilities[i]=std::exp(p[i]-maximum));
    for (auto& value:probabilities) value/=total;
    return probabilities;
}

struct AtlasV4Result {
    std::array<std::vector<float>,3> logits;
    float valid_probability=0, color_probability=0, direction_probability=0;
    int raw_color=0, raw_direction=0;
    std::string color="unknown", direction="unknown";
    // [P3] Loose direction (lower threshold), used only by the round->arrow
    // guard correction; the public direction keeps the strict 0.90 gate.
    std::string direction_loose="unknown";
    double infer_ms=0;
    cv::Rect roi;
    size_t candidate_index=0;
    std::string original_label;
};

inline void atlas_v4_interpret(AtlasV4Result& result, float native_w, float native_h) {
    const auto pv=atlas_v4_prob(result.logits[0].data(),2);
    const auto pc=atlas_v4_prob(result.logits[1].data(),4);
    const auto pd=atlas_v4_prob(result.logits[2].data(),4);
    const char* colors[]={"red","yellow","green","off"};
    const char* directions[]={"round","forward","left","right"};
    result.raw_color=std::max_element(pc.begin(),pc.end())-pc.begin();
    result.raw_direction=std::max_element(pd.begin(),pd.end())-pd.begin();
    result.valid_probability=pv[1]; result.color_probability=pc[result.raw_color];
    result.direction_probability=pd[result.raw_direction];
    const bool valid=pv[1]>=.8f;
    result.color=valid && result.color_probability>=.7f ? colors[result.raw_color] : "unknown";
    result.direction=valid && std::min(native_w,native_h)>=8 &&
        result.direction_probability>=.90f ? directions[result.raw_direction] : "unknown";
    // [P3] Same argmax, lower threshold (P3_ATLAS_DIR_LOOSE, default 0.65).
    static const float kLooseDirConf = []() {
        const char* v=std::getenv("P3_ATLAS_DIR_LOOSE");
        const double d=v?std::atof(v):0.65;
        return static_cast<float>(d>0.0 && d<1.0 ? d : 0.65);
    }();
    result.direction_loose=valid && std::min(native_w,native_h)>=8 &&
        result.direction_probability>=kLooseDirConf ?
        directions[result.raw_direction] : "unknown";
    // Forward/right are displayed as independent strings only, not mapped to
    // legacy seven-class indices. No decision authorization is granted here.
}

class AtlasV4Runner {
    rknn_context context_=0;
public:
    explicit AtlasV4Runner(const std::string& path) {
        std::ifstream stream(path,std::ios::binary|std::ios::ate);
        if (!stream) throw std::runtime_error("ATLAS v4 model missing");
        const auto length=stream.tellg();
        if (length<=0 || length>64*1024*1024) throw std::runtime_error("ATLAS v4 model size invalid");
        std::vector<char> bytes(static_cast<size_t>(length)); stream.seekg(0);
        if (!stream.read(bytes.data(),bytes.size())) throw std::runtime_error("ATLAS v4 model read failed");
        if (rknn_init(&context_,bytes.data(),bytes.size(),0,nullptr)!=RKNN_SUCC)
            throw std::runtime_error("ATLAS v4 rknn_init failed");
        try {
            rknn_input_output_num count{};
            if (rknn_query(context_,RKNN_QUERY_IN_OUT_NUM,&count,sizeof(count))!=RKNN_SUCC ||
                count.n_input!=1 || count.n_output!=3) throw std::runtime_error("ATLAS v4 head count mismatch");
            rknn_tensor_attr input{};
            if (rknn_query(context_,RKNN_QUERY_INPUT_ATTR,&input,sizeof(input))!=RKNN_SUCC ||
                input.n_elems!=128*128*3) throw std::runtime_error("ATLAS v4 input shape mismatch");
            const int sizes[]={2,4,4};
            const char* names[]={"valid_logits","color_logits","direction_logits"};
            for (unsigned i=0;i<3;++i) {
                rknn_tensor_attr attr{}; attr.index=i;
                if (rknn_query(context_,RKNN_QUERY_OUTPUT_ATTR,&attr,sizeof(attr))!=RKNN_SUCC ||
                    attr.n_elems!=static_cast<unsigned>(sizes[i]) || std::string(attr.name)!=names[i])
                    throw std::runtime_error("ATLAS v4 output schema/order mismatch");
                std::printf("[atlas-v4] head=%u name=%s elems=%u\n",i,attr.name,attr.n_elems);
            }
            std::printf("[atlas-v4] FP16 ready; standalone authorization=OFF; offline_screen_passed=false; model=%s\n",path.c_str());
            std::fflush(stdout);
        } catch (...) { rknn_destroy(context_); context_=0; throw; }
    }
    ~AtlasV4Runner() { if (context_) rknn_destroy(context_); }
    AtlasV4Runner(const AtlasV4Runner&)=delete;
    AtlasV4Runner& operator=(const AtlasV4Runner&)=delete;
    AtlasV4Result infer(const cv::Mat& bgr, float native_w, float native_h) {
        cv::Mat prepared=atlas_v4_prepare(bgr), rgb;
        cv::cvtColor(prepared,rgb,cv::COLOR_BGR2RGB);
        rknn_input input{}; input.index=0; input.type=RKNN_TENSOR_UINT8;
        input.fmt=RKNN_TENSOR_NHWC; input.size=rgb.total()*rgb.elemSize(); input.buf=rgb.data;
        const auto start=std::chrono::steady_clock::now();
        if (rknn_inputs_set(context_,1,&input)!=RKNN_SUCC || rknn_run(context_,nullptr)!=RKNN_SUCC)
            throw std::runtime_error("ATLAS v4 inference failed");
        rknn_output outputs[3]{};
        for (unsigned i=0;i<3;++i) { outputs[i].index=i; outputs[i].want_float=1; }
        if (rknn_outputs_get(context_,3,outputs,nullptr)!=RKNN_SUCC)
            throw std::runtime_error("ATLAS v4 outputs_get failed");
        AtlasV4Result result; const int sizes[]={2,4,4};
        try {
            for (unsigned i=0;i<3;++i) {
                if (!outputs[i].buf || outputs[i].size<sizes[i]*sizeof(float))
                    throw std::runtime_error("ATLAS v4 output buffer invalid");
                const auto p=static_cast<const float*>(outputs[i].buf);
                result.logits[i].assign(p,p+sizes[i]);
            }
            atlas_v4_interpret(result,native_w,native_h);
        } catch (...) { rknn_outputs_release(context_,3,outputs); throw; }
        rknn_outputs_release(context_,3,outputs);
        result.infer_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        return result;
    }
};

class AtlasV4Shadow {
    bool enabled_=false;
    AtlasV4Runner* runner_=nullptr;
    std::ofstream csv_;
    std::vector<AtlasV4Result> last_;
    double capture_ms_=0;
    int last_frame_=-1;
    uint64_t run_id_=0;
    bool decision_=false;
public:
    AtlasV4Shadow() {
        const char* flag=std::getenv("P3_ATLAS_V4_SHADOW");
        enabled_=flag && std::string(flag)=="1";
        const char* decision=std::getenv("P3_ATLAS_V4_DECISION");
        decision_=decision && std::string(decision)=="1";
        if (decision_ && !enabled_) throw std::runtime_error("ATLAS decision requires attributes enabled");
    }
    ~AtlasV4Shadow() { delete runner_; }
    void reset() { last_.clear(); capture_ms_=0; last_frame_=-1; run_id_=0; }
    template<class D> void constrain(D& d,size_t index,uint64_t run,const std::string& original) {
        if (!decision_) return;
        const AtlasV4Result* match=nullptr;
        if (run==run_id_ && capture_ms_>0)
            for (const auto& r:last_) if (r.candidate_index==index) { match=&r; break; }
        const std::string before=d.label;
        d.label=atlas_guard_label(original,before,
            match ? match->color : "unknown",match ? match->direction : "unknown",
            match!=nullptr,match ? match->direction_loose : "unknown");
        if (d.label!=before) d.cls=p3_light::class_index_of(d.label);
        std::printf("[atlas-guard] run=%llu candidate=%zu legacy=%s result=%s matched=%d simulation_only=1\n",
            static_cast<unsigned long long>(run),index,before.c_str(),d.label.c_str(),match!=nullptr);
    }
    template<class D> void observe(const cv::Mat& input,const std::vector<D>& detections,
                                   uint64_t run_id,double capture_ms,int frame) {
        if (!enabled_ || capture_ms<=0) return; // Do not classify/draw warmup images.
        if (!runner_) {
            const char* model=std::getenv("P3_ATLAS_V4_MODEL");
            const char* log=std::getenv("P3_ATLAS_V4_CSV");
            if (!model || !log) throw std::runtime_error("ATLAS v4 model/CSV environment missing");
            runner_=new AtlasV4Runner(model); csv_.open(log,std::ios::out);
            if (!csv_) throw std::runtime_error("ATLAS v4 CSV open failed");
            csv_ << "frame,locator_run,capture_ms,x,y,w,h,legacy_label,color,direction,valid_p,color_p,direction_p,infer_ms,decision_authorized\n";
        }
        reset(); capture_ms_=capture_ms; last_frame_=frame; run_id_=run_id;
        const int width=input.cols/2;
        for (size_t i=0;i<std::min<size_t>(detections.size(),2);++i) {
            const auto& d=detections[i];
            if (!std::isfinite(d.x1)||!std::isfinite(d.x2)||!std::isfinite(d.y1)||!std::isfinite(d.y2) ||
                d.x2<=d.x1 || d.y2<=d.y1 || d.x2<=0 || d.y2<=0 ||
                d.x1>=width || d.y1>=input.rows) continue;
            const int bw=std::max(1,int(std::ceil(d.x2-d.x1))), bh=std::max(1,int(std::ceil(d.y2-d.y1)));
            const int mx=std::max(2,bw/5),my=std::max(2,bh/5);
            const int x1=std::clamp(int(std::floor(d.x1))-mx,0,width-1), y1=std::clamp(int(std::floor(d.y1))-my,0,input.rows-1);
            const int x2=std::clamp(int(std::ceil(d.x2))+mx,x1+1,width), y2=std::clamp(int(std::ceil(d.y2))+my,y1+1,input.rows);
            const cv::Rect roi(x1,y1,x2-x1,y2-y1);
            auto result=runner_->infer(input(roi),d.x2-d.x1,d.y2-d.y1); result.roi=roi;
            result.candidate_index=i; result.original_label=d.label;
            csv_ << frame << ',' << run_id << ',' << capture_ms << ',' << x1 << ',' << y1 << ',' << roi.width << ',' << roi.height << ','
                 << d.label << ',' << result.color << ',' << result.direction << ',' << result.valid_probability << ','
                 << result.color_probability << ',' << result.direction_probability << ',' << result.infer_ms << ",0\n";
            std::printf("[atlas-v4] frame=%d run=%llu color=%s direction=%s v=%.3f c=%.3f d=%.3f ms=%.2f control=OFF rawc=%d rawd=%d loose=%s\n",
                frame,static_cast<unsigned long long>(run_id),result.color.c_str(),result.direction.c_str(),
                result.valid_probability,result.color_probability,result.direction_probability,result.infer_ms,
                result.raw_color,result.raw_direction,result.direction_loose.c_str());
            last_.push_back(result);
        }
        csv_.flush(); std::fflush(stdout);
    }
    void draw(cv::Mat& image,int frame,bool locator_ok,bool locator_failed,double capture_ms) {
        if (!enabled_ || capture_ms<=0) return;
        // [ATLAS-DRAW 2026-10-21] 调试面板会遮住右目顶部 640x78（深色矩形+
        // 三行诊断字）。显示可单独关闭（P3_ATLAS_V4_DRAW=0）；observe/
        // constrain/CSV 与灯决策完全不受影响。
        if (const char* draw_flag = std::getenv("P3_ATLAS_V4_DRAW"))
            if (std::string(draw_flag) == "0") return;
        const int x=image.cols/2+5;
        cv::rectangle(image,cv::Rect(image.cols/2,0,image.cols/2,78),cv::Scalar(20,20,20),cv::FILLED);
        cv::putText(image,decision_ ? "ATLAS v4 SIM GUARD / NO ACTUATORS" : "ATLAS v4 EXPERIMENT / NO CONTROL",cv::Point(x,17),cv::FONT_HERSHEY_SIMPLEX,.44,cv::Scalar(0,200,255),1);
        const bool fresh=frame==last_frame_ && locator_ok;
        const bool usable=last_frame_>=0 && !locator_failed && capture_ms>=capture_ms_ && capture_ms-capture_ms_<=500 && frame-last_frame_<=2;
        std::string status=locator_failed ? "locator failed; UNKNOWN" :
            (!usable ? "no fresh attribute result" : (fresh ? "fresh" : "held (not a new observation)"));
        cv::putText(image,status,cv::Point(x,34),cv::FONT_HERSHEY_SIMPLEX,.44,cv::Scalar(220,220,220),1);
        if (!usable) return;
        if (last_.empty()) cv::putText(image,"locator found no lamp",cv::Point(x,53),cv::FONT_HERSHEY_SIMPLEX,.44,cv::Scalar(220,220,220),1);
        for (size_t i=0;i<last_.size();++i) {
            const auto& r=last_[i]; char line[160];
            std::snprintf(line,sizeof(line),"V4 %s / %s  c=%.2f d=%.2f  %.1fms",r.color.c_str(),r.direction.c_str(),r.color_probability,r.direction_probability,r.infer_ms);
            cv::putText(image,line,cv::Point(x,53+int(i)*18),cv::FONT_HERSHEY_SIMPLEX,.43,cv::Scalar(0,200,255),1);
            // Do not attach held attributes to an old box on a moving frame.
            if (fresh) {
                cv::rectangle(image,r.roi,cv::Scalar(255,0,255),1);
                cv::putText(image,"V4 "+r.color+"/"+r.direction,cv::Point(r.roi.x,std::min(image.rows-3,r.roi.y+r.roi.height+13)),
                    cv::FONT_HERSHEY_SIMPLEX,.4,cv::Scalar(255,0,255),1);
            }
        }
    }
};

inline AtlasV4Shadow& atlas_v4_shadow() { static AtlasV4Shadow shadow; return shadow; }
#endif
