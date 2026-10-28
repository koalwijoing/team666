#include "crosswalk_person.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace p3_crosswalk {
namespace {
bool valid_time(double source, double now, double ttl) {
    return std::isfinite(source) && std::isfinite(now) && source > 0 &&
           now >= source && now - source <= ttl;
}
double median(std::vector<double> values) {
    if (values.empty()) return 0;
    std::sort(values.begin(), values.end());
    return values[values.size()/2];
}
double overlap(const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
    if (a.empty() || b.empty()) return 0;
    const cv::Rect x = cv::boundingRect(a), y = cv::boundingRect(b);
    return static_cast<double>((x & y).area()) / std::max(1, (x | y).area());
}
struct Stripe {
    cv::Point2f center;
    cv::Rect bounds;
    std::vector<cv::Point> contour;
    double area = 0;
    cv::Point2f axis;
    double aspect = 0;
};

bool perspective_consistent(const std::vector<int>& group,const std::vector<Stripe>& candidates,int width,int height) {
    cv::Matx22d normal(0,0,0,0);
    cv::Vec2d rhs(0,0);
    int useful=0;
    double center_y=0,mean_h=0;
    for(int index:group) {
        const auto& stripe=candidates[index];
        center_y+=stripe.center.y;mean_h+=stripe.bounds.height;
        if(stripe.aspect<1.3)continue; // a near-square has no stable long axis
        const double length=cv::norm(stripe.axis);
        const double nx=stripe.axis.y/length,ny=-stripe.axis.x/length;
        const double b=nx*stripe.center.x+ny*stripe.center.y;
        normal(0,0)+=nx*nx;normal(0,1)+=nx*ny;
        normal(1,0)+=nx*ny;normal(1,1)+=ny*ny;
        rhs[0]+=nx*b;rhs[1]+=ny*b;++useful;
    }
    if(useful<3)return false;
    const double det=normal(0,0)*normal(1,1)-normal(0,1)*normal(1,0);
    if(det<.006*useful*useful)return false; // parallel white vehicle roofs
    const cv::Vec2d vanishing=normal.inv()*rhs;
    center_y/=group.size();mean_h/=group.size();
    if(vanishing[0]<-width*.5||vanishing[0]>width*1.5||
       vanishing[1]<-height||vanishing[1]>center_y-mean_h*.6)return false;
    int aligned=0;
    for(int index:group) {
        const auto& s=candidates[index];
        if(s.aspect<1.3){++aligned;continue;}
        const cv::Point2f ray(s.center.x-vanishing[0],s.center.y-vanishing[1]);
        const double cosine=std::abs(ray.dot(s.axis))/std::max(1.,cv::norm(ray)*cv::norm(s.axis));
        if(cosine>=.88)++aligned;
    }
    return aligned>=cvCeil(group.size()*.80);
}

void geometry(const Observation& input, const Config& cfg, Result& result) {
    const double scale = std::min(1.0, static_cast<double>(cfg.analysis_width) / input.left_bgr.cols);
    cv::Mat small, gray, smooth, white, neutral;
    cv::resize(input.left_bgr, small, {}, scale, scale, cv::INTER_AREA);
    cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);
    cv::boxFilter(gray, smooth, -1, cv::Size(27, 27), {-1,-1}, true, cv::BORDER_REPLICATE);
    white = cv::Mat::zeros(gray.size(), CV_8U);
    neutral = cv::Mat::zeros(gray.size(), CV_8U);
    // Fixed forward-camera layout: reserve the lower road band. Repeated
    // bright facade/roof fragments near the horizon are not road evidence.
    const int y0 = static_cast<int>(gray.rows * .60);
    const int y1 = static_cast<int>(gray.rows * .98);
    for (int y = y0; y < y1; ++y) {
        const auto* rgb = small.ptr<cv::Vec3b>(y);
        const auto* g = gray.ptr<unsigned char>(y);
        const auto* avg = smooth.ptr<unsigned char>(y);
        auto* out = white.ptr<unsigned char>(y);
        auto* hint_white = neutral.ptr<unsigned char>(y);
        for (int x = 4; x < gray.cols - 4; ++x) {
            const int lo = std::min({rgb[x][0], rgb[x][1], rgb[x][2]});
            const int hi = std::max({rgb[x][0], rgb[x][1], rgb[x][2]});
            if(g[x]>=105 && hi-lo<=85 && lo>=70)hint_white[x]=255;
            if (g[x] >= 125 && hi-lo <= 85 && lo >= 85 &&
                static_cast<int>(g[x])-avg[x] >= 12) out[x] = 255;
        }
    }
    cv::morphologyEx(white, white, cv::MORPH_CLOSE,
                    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
    const auto extract = [&](const cv::Mat& binary, bool twin_guided) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<Stripe> candidates;
    for (const auto& c : contours) {
        const double area = cv::contourArea(c);
        const cv::Rect bounds = cv::boundingRect(c);
        if (area < 12 || area > gray.total()*.045 || bounds.height < 5 ||
            bounds.height > gray.rows*.30 || bounds.width < 3 ||
            bounds.width > gray.cols*.55 || bounds.y < y0) continue;
        const cv::RotatedRect rr = cv::minAreaRect(c);
        const double shorter = std::min(rr.size.width, rr.size.height);
        const double longer = std::max(rr.size.width, rr.size.height);
        if (shorter < 2 || longer/std::max(1.0,shorter) < 1.12 ||
            longer/shorter > 50 || area / std::max(1.f, rr.size.area()) < .30) continue;
        cv::Point2f corners[4]; rr.points(corners);
        cv::Point2f axis = corners[1]-corners[0];
        if (cv::norm(corners[2]-corners[1]) > cv::norm(axis)) axis=corners[2]-corners[1];
        std::vector<cv::Point> hull;cv::convexHull(c,hull);
        if(area/std::max(1.,cv::contourArea(hull))<.60)continue;
        candidates.push_back({rr.center,bounds,c,area,axis,longer/shorter});
    }
    if (candidates.size() > 80) {
        std::sort(candidates.begin(),candidates.end(),[](const Stripe& a,const Stripe& b){return a.area>b.area;});
        candidates.resize(80);
    }
    std::vector<int> best;
    double best_score = 0;
    for (std::size_t a=0; a<candidates.size(); ++a) {
        for (std::size_t b=a+1; b<candidates.size(); ++b) {
            const cv::Point2f delta = candidates[b].center-candidates[a].center;
            if (std::abs(delta.x)<18 || std::abs(delta.y/delta.x)>.75) continue;
            const double slope=delta.y/delta.x;
            const double base=candidates[a].center.y-slope*candidates[a].center.x;
            std::vector<int> group;
            const double base_height=(candidates[a].bounds.height+candidates[b].bounds.height)*.5;
            for (std::size_t k=0;k<candidates.size();++k) {
                const auto& s=candidates[k];
                if (s.bounds.height < base_height*.45 || s.bounds.height > base_height*2.2) continue;
                if (std::abs(s.center.y-(base+slope*s.center.x)) <= std::max(3.,base_height*.28))
                    group.push_back(static_cast<int>(k));
            }
            if (static_cast<int>(group.size())<cfg.min_stripes) continue;
            std::vector<double> lengths;
            for(int k:group)lengths.push_back(candidates[k].bounds.height);
            const double median_height=median(lengths);
            group.erase(std::remove_if(group.begin(),group.end(),[&](int k){
                return candidates[k].bounds.height<median_height*.55||
                       candidates[k].bounds.height>median_height*1.8;
            }),group.end());
            if (static_cast<int>(group.size())<cfg.min_stripes) continue;
            std::sort(group.begin(),group.end(),[&](int i,int j){return candidates[i].center.x<candidates[j].center.x;});
            const double span=candidates[group.back()].center.x-candidates[group.front()].center.x;
            if (span < gray.cols*.22) continue;
            std::vector<double> gaps, heights;
            double width_sum=0;
            bool duplicate=false;
            for(std::size_t k=0;k<group.size();++k) {
                const auto& s=candidates[group[k]];
                heights.push_back(s.bounds.height);
                width_sum += s.area/s.bounds.height;
                if(k) {
                    const double gap=s.center.x-candidates[group[k-1]].center.x;
                    if(gap<4) duplicate=true;
                    gaps.push_back(gap);
                }
            }
            if(duplicate || width_sum/span<.16 || width_sum/span>.86) continue;
            const double mid_gap=median(gaps);
            if(*std::max_element(gaps.begin(),gaps.end())>std::max(mid_gap*3.3,gray.cols*.23)) continue;
            // Painted stripes must alternate with visible darker road gaps.
            int contrast_hits=0;
            for(std::size_t k=1;k<group.size();++k) {
                const auto& p=candidates[group[k-1]];
                const auto& q=candidates[group[k]];
                int x=cvRound((p.center.x+q.center.x)*.5);
                int y=cvRound(base+slope*x);
                if(x<2||x>=gray.cols-2||y<2||y>=gray.rows-2)continue;
                const double gap_gray=cv::mean(gray(cv::Rect(x-1,y-1,3,3)))[0];
                const int px=std::clamp(cvRound(p.center.x),0,gray.cols-1);
                const int py=std::clamp(cvRound(p.center.y),0,gray.rows-1);
                const int qx=std::clamp(cvRound(q.center.x),0,gray.cols-1);
                const int qy=std::clamp(cvRound(q.center.y),0,gray.rows-1);
                const double paint=(gray.at<unsigned char>(py,px)+gray.at<unsigned char>(qy,qx))*.5;
                if(paint-gap_gray>=13)++contrast_hits;
            }
            if(contrast_hits < std::max(2,cvCeil(gaps.size()*.60)))continue;
            if(!perspective_consistent(group,candidates,gray.cols,gray.rows))continue;
            const double score=group.size()+span/gray.cols+contrast_hits*.2;
            if(score>best_score){best_score=score;best=group;}
        }
    }
    if(best.empty() && twin_guided) {
        // An oblique crossing can leave only a clipped bank of long,
        // near-horizontal bars. Require three separately observed thin paint
        // bands, common visible extent, regular gaps and dark road between
        // them; this branch also requires the current Twin marking mask.
        for(std::size_t anchor=0;anchor<candidates.size();++anchor) {
            const auto& a=candidates[anchor];
            if(a.aspect<8||a.bounds.width<gray.cols*.08||a.bounds.y<gray.rows*.60||
               std::abs(a.axis.x)<1||std::abs(a.axis.y/a.axis.x)>.40)continue;
            std::vector<int> group;
            int x0=a.bounds.x,x1=a.bounds.x+a.bounds.width;
            for(std::size_t k=0;k<candidates.size();++k) {
                const auto& b=candidates[k];
                if(b.aspect<8||b.bounds.width<gray.cols*.08||b.bounds.y<gray.rows*.60||
                   std::abs(b.axis.x)<1||std::abs(b.axis.y/b.axis.x)>.40)continue;
                const double cosine=std::abs(a.axis.dot(b.axis))/std::max(1.,cv::norm(a.axis)*cv::norm(b.axis));
                const int common=std::min(a.bounds.x+a.bounds.width,b.bounds.x+b.bounds.width)-std::max(a.bounds.x,b.bounds.x);
                if(cosine<.97||common<std::min(a.bounds.width,b.bounds.width)*.50)continue;
                group.push_back(static_cast<int>(k));
                x0=std::max(x0,b.bounds.x);x1=std::min(x1,b.bounds.x+b.bounds.width);
            }
            if(group.size()<3||x1-x0<gray.cols*.08)continue;
            std::sort(group.begin(),group.end(),[&](int i,int j){return candidates[i].center.y<candidates[j].center.y;});
            if(candidates[group.back()].center.y-candidates[group.front()].center.y<gray.rows*.075)continue;
            std::vector<double> gaps;
            int contrast=0;bool duplicate=false;
            for(std::size_t k=1;k<group.size();++k) {
                const auto& p=candidates[group[k-1]];const auto& q=candidates[group[k]];
                const double gap=q.center.y-p.center.y;gaps.push_back(gap);
                if(gap<4)duplicate=true;
                const int x=(x0+x1)/2;
                const double p_at_x=p.center.y+(x-p.center.x)*p.axis.y/p.axis.x;
                const double q_at_x=q.center.y+(x-q.center.x)*q.axis.y/q.axis.x;
                const int py=std::clamp(cvRound(p_at_x),0,gray.rows-1),qy=std::clamp(cvRound(q_at_x),0,gray.rows-1);
                const int mid=std::clamp(cvRound((p_at_x+q_at_x)*.5),0,gray.rows-1);
                if((gray.at<unsigned char>(py,x)+gray.at<unsigned char>(qy,x))*.5-gray.at<unsigned char>(mid,x)>=12)++contrast;
            }
            const double low=*std::min_element(gaps.begin(),gaps.end()),high=*std::max_element(gaps.begin(),gaps.end());
            if(duplicate||high>low*2.2||contrast<cvCeil(gaps.size()*.75))continue;
            const double score=group.size()+static_cast<double>(x1-x0)/gray.cols+contrast*.2;
            if(score>best_score){best_score=score;best=group;}
        }
    }
    if(best.empty()) return;
    std::vector<cv::Point> all;
    cv::Mat stripe_pixels=cv::Mat::zeros(gray.size(),CV_8U);
    for(int i:best) {
        std::vector<cv::Point> polygon;
        cv::approxPolyDP(candidates[i].contour,polygon,1.3,true);
        cv::drawContours(stripe_pixels, std::vector<std::vector<cv::Point>>{candidates[i].contour},0,255,cv::FILLED);
        std::vector<cv::Point> full;
        for(const auto& p:polygon)full.emplace_back(cvRound(p.x/scale),cvRound(p.y/scale));
        all.insert(all.end(),full.begin(),full.end());
        result.stripes.push_back(std::move(full));
    }
    cv::convexHull(all,result.region);
    result.geometry_score=best_score;
    if(!input.twin_mask.empty() && input.twin_mask.type()==CV_8UC1) {
        cv::Mat mask, matched;
        cv::resize(input.twin_mask,mask,gray.size(),0,0,cv::INTER_NEAREST);
        cv::bitwise_and(mask>0,stripe_pixels,matched);
        result.twin_support=static_cast<double>(cv::countNonZero(matched))/std::max(1,cv::countNonZero(stripe_pixels));
    }
    };
    // On bright roads, the low floor can join separate paint bars through
    // pavement highlights. Test their brighter cores first, retaining every
    // repeat, contrast and perspective check; dim paint uses the original path.
    cv::Mat bright;
    cv::bitwise_and(white,gray>=195,bright);
    extract(bright,false);
    if(result.region.empty())extract(white,false);
    if(result.region.empty() && !input.twin_mask.empty() && input.twin_mask.type()==CV_8UC1) {
        // The current successful Twin observation may separate painted bars
        // which RGB thresholding merged with a vehicle or bright pavement.
        // It remains a geometric repeat/contrast test, not a crosswalk class.
        cv::Mat hint,guided;
        cv::resize(input.twin_mask,hint,white.size(),0,0,cv::INTER_NEAREST);
        cv::bitwise_and(neutral,hint,guided);
        extract(guided,true);
    }
}
}  // namespace

Detector::Detector(Config config):config_(config) {
    if(config.analysis_width<160||config.min_stripes<3||config.confirm_observations<1||
       !std::isfinite(config.ttl_ms)||config.ttl_ms<=0)throw std::invalid_argument("crosswalk config");
}
void Detector::reset(){result_=Result{};}
const Result& Detector::update(const Observation& input) {
    if(input.delivery_id==result_.delivery_id && input.delivery_id!=0) return result_;
    const auto started=std::chrono::steady_clock::now();
    Result next;
    next.delivery_id=input.delivery_id;
    next.source_ms=input.source_ms;
    next.width=input.left_bgr.cols;next.height=input.left_bgr.rows;
    if(!input.delivery_id||input.left_bgr.empty()||input.left_bgr.type()!=CV_8UC3||
       input.left_bgr.cols<160||input.left_bgr.rows<90||
       !valid_time(input.source_ms,input.now_ms,config_.ttl_ms)) {
        result_=std::move(next);return result_;
    }
    if(const char* directory=std::getenv("P3_CROSSWALK_DUMP_DIR")) {
        if(input.delivery_id<=31 && (input.delivery_id-1)%10==0) {
            const std::string base=std::string(directory)+"/frame_"+std::to_string(input.delivery_id);
            if(!cv::imwrite(base+"_left.png",input.left_bgr))throw std::runtime_error("crosswalk diagnostic image write failed");
            if(!input.twin_mask.empty())cv::imwrite(base+"_twin.png",input.twin_mask);
        }
    }
    geometry(input,config_,next);
    next.state=next.region.empty()?State::NotDetected:State::Candidate;
    if(!next.region.empty()) {
        next.confirmations=1;
        if(input.delivery_id>result_.delivery_id && input.source_ms>result_.source_ms &&
           valid_time(result_.source_ms,input.source_ms,config_.ttl_ms) &&
           overlap(next.region,result_.region)>.25)
            next.confirmations=std::min(config_.confirm_observations,result_.confirmations+1);
        if(next.confirmations>=config_.confirm_observations)next.state=State::Confirmed;
    }
    next.elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    result_=std::move(next);return result_;
}
Result Detector::view(std::uint64_t id,double now) const {
    Result result=result_;
    if(result.state!=State::Unavailable &&
       (!valid_time(result.source_ms,now,config_.ttl_ms)||id<result.delivery_id||
        id-result.delivery_id>config_.max_age_frames)) {
        result.state=State::Stale;result.region.clear();result.stripes.clear();
    } else if(result.state!=State::Unavailable && id!=result.delivery_id) {
        result.state=State::Held;
    }
    return result;
}
PredictiveDetector::PredictiveDetector(Config config):config_(config),detector_(config) {
    if(!std::isfinite(config.prediction_max_ms)||config.prediction_max_ms<=0||
       config.prediction_max_ms>config.ttl_ms||!config.prediction_max_frames)
        throw std::invalid_argument("crosswalk prediction config");
}
void PredictiveDetector::reset() {
    detector_.reset();tracking_=false;measurement_=Result{};result_=Result{};
    axes_={};filter_ms_=last_input_ms_=0;last_input_id_=0;previous_scene_.release();
}
PredictiveDetector::Axis PredictiveDetector::advance(Axis a,double dt) {
    const double acceleration_variance=.35*.35;
    a.value+=a.velocity*dt;
    a.p00+=2*dt*a.p01+dt*dt*a.p11+acceleration_variance*dt*dt*dt*dt*.25;
    a.p01+=dt*a.p11+acceleration_variance*dt*dt*dt*.5;
    a.p11+=acceleration_variance*dt*dt;
    return a;
}
void PredictiveDetector::seed(const Result& m) {
    const auto b=cv::boundingRect(m.region);
    const std::array<double,4> z{{(b.x+b.width*.5)/m.width,(b.y+b.height*.5)/m.height,
                                double(b.width)/m.width,double(b.height)/m.height}};
    for(int i=0;i<4;++i)axes_[i]={z[i],0,i<2?.003*.003:.008*.008,0,.1*.1};
    filter_ms_=m.source_ms;measurement_=m;tracking_=true;
}
Result PredictiveDetector::project(std::uint64_t id,double source,double now) const {
    Result out;out.state=State::Stale;
    out.width=measurement_.width;out.height=measurement_.height;
    out.delivery_id=measurement_.delivery_id;out.source_ms=measurement_.source_ms;
    if(!tracking_||id<=measurement_.delivery_id||id-measurement_.delivery_id>config_.prediction_max_frames||
       !valid_time(measurement_.source_ms,source,config_.prediction_max_ms)||
       !valid_time(measurement_.source_ms,now,config_.prediction_max_ms)||now<source)return out;
    const double dt=(source-filter_ms_)/1000.;
    if(dt<0)return out;
    std::array<Axis,4> projected;
    for(int i=0;i<4;++i)projected[i]=advance(axes_[i],dt);
    const auto old=cv::boundingRect(measurement_.region);
    const double width=projected[2].value*out.width,height=projected[3].value*out.height;
    const double cx=projected[0].value*out.width,cy=projected[1].value*out.height;
    const double sigma=std::max(std::sqrt(std::max(0.,projected[0].p00))*out.width,
                               std::sqrt(std::max(0.,projected[1].p00))*out.height);
    if(!std::isfinite(cx+cy+width+height+sigma)||width<4||height<4||
       width/old.width<.65||width/old.width>1.6||height/old.height<.65||height/old.height>1.6||
       sigma>out.width*.05||std::abs(cx-(old.x+old.width*.5))>out.width*.15||
       std::abs(cy-(old.y+old.height*.5))>out.height*.15)return out;
    for(const auto& p:measurement_.region) {
        const double x=cx+(p.x-(old.x+old.width*.5))*width/old.width;
        const double y=cy+(p.y-(old.y+old.height*.5))*height/old.height;
        // Leaving the image is a reason to withdraw, not to draw a clipped
        // polygon that appears more certain than the actual projection.
        if(x<0||x>=out.width||y<0||y>=out.height){out.region.clear();return out;}
        out.region.emplace_back(std::clamp(cvRound(x),0,out.width-1),std::clamp(cvRound(y),0,out.height-1));
    }
    out.state=State::Predicted;out.projection_ms=source;
    out.prediction_age_ms=source-measurement_.source_ms;out.prediction_sigma_px=sigma;
    return out;
}
const Result& PredictiveDetector::update(const Observation& input) {
    if(!config_.prediction_enabled){result_=detector_.update(input);return result_;}
    if(input.delivery_id&&input.delivery_id==last_input_id_)return result_;
    const auto started=std::chrono::steady_clock::now();
    const bool valid=input.delivery_id&&!input.left_bgr.empty()&&input.left_bgr.type()==CV_8UC3&&
        input.left_bgr.cols>=160&&input.left_bgr.rows>=90&&valid_time(input.source_ms,input.now_ms,config_.ttl_ms);
    if(!valid||(last_input_id_&&(input.delivery_id<last_input_id_||input.source_ms<=last_input_ms_))) {
        reset();return result_;
    }
    cv::Mat scene,gray;
    cv::resize(input.left_bgr,scene,{32,18},0,0,cv::INTER_AREA);
    cv::cvtColor(scene,gray,cv::COLOR_BGR2GRAY);
    cv::Mat difference;
    bool cut=false;
    if(!previous_scene_.empty()) {
        cv::absdiff(gray,previous_scene_,difference);
        cut=cv::mean(difference)[0]>32;
    }
    const bool resized=measurement_.width && (measurement_.width!=input.left_bgr.cols||measurement_.height!=input.left_bgr.rows);
    if(cut||resized||(last_input_ms_&&input.source_ms-last_input_ms_>config_.prediction_max_ms)) {
        detector_.reset();tracking_=false;measurement_=Result{};
    }
    previous_scene_=gray;last_input_id_=input.delivery_id;last_input_ms_=input.source_ms;
    Result raw=detector_.update(input);
    if(!raw.region.empty()) {
        if(tracking_&&valid_time(measurement_.source_ms,input.source_ms,config_.prediction_max_ms)) {
            const auto b=cv::boundingRect(raw.region);
            const std::array<double,4> z{{(b.x+b.width*.5)/raw.width,(b.y+b.height*.5)/raw.height,
                                        double(b.width)/raw.width,double(b.height)/raw.height}};
            std::array<Axis,4> predicted;
            double distance=0;
            for(int i=0;i<4;++i) {
                predicted[i]=advance(axes_[i],(input.source_ms-filter_ms_)/1000.);
                const double variance=i<2?.003*.003:.008*.008;
                const double innovation=z[i]-predicted[i].value;
                distance+=innovation*innovation/(predicted[i].p00+variance);
            }
            // Reject a different crossing or a fragment whose size jumps.
            if(distance<=25 && overlap(raw.region,measurement_.region)>.12) {
                for(int i=0;i<4;++i) {
                    auto a=predicted[i];const double variance=i<2?.003*.003:.008*.008;
                    const double s=a.p00+variance,k0=a.p00/s,k1=a.p01/s,innovation=z[i]-a.value;
                    a.value+=k0*innovation;a.velocity+=k1*innovation;
                    // Joseph-equivalent scalar covariance update.
                    a.p11-=a.p01*a.p01/s;a.p01*=1-k0;a.p00*=1-k0;
                    axes_[i]=a;
                }
                filter_ms_=input.source_ms;raw.state=State::Confirmed;
                raw.confirmations=config_.confirm_observations;measurement_=raw;
            } else {tracking_=false;raw.state=State::Candidate;raw.confirmations=1;}
        } else {
            tracking_=false;
            if(raw.state==State::Confirmed)seed(raw);
        }
        result_=raw;
    } else if(tracking_) {
        result_=project(input.delivery_id,input.source_ms,input.now_ms);
        if(result_.state!=State::Predicted){tracking_=false;result_=raw;}
    } else result_=raw;
    result_.elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    return result_;
}
Result PredictiveDetector::view(std::uint64_t id,double now,double source) const {
    if(!config_.prediction_enabled)return detector_.view(id,now);
    if(id<last_input_id_||!std::isfinite(source)||now<source||
       (id>last_input_id_&&source<=last_input_ms_)) {
        Result stale=result_;stale.state=State::Stale;stale.region.clear();stale.stripes.clear();return stale;
    }
    if(id==last_input_id_) {
        const double ttl=result_.state==State::Predicted?config_.prediction_max_ms:config_.ttl_ms;
        if(valid_time(result_.source_ms,now,ttl))return result_;
    } else if(tracking_)return project(id,source,now);
    return detector_.view(id,now);
}

std::vector<PersonAssociation> associate(const Result& crosswalk,const std::vector<Person>& persons,double now,double ttl) {
    std::vector<PersonAssociation> output;
    for(const auto& p:persons) {
        PersonAssociation item;
        item.person=p;item.foot={p.box.x+p.box.width*.5f,p.box.y+p.box.height};
        const bool valid_box=std::isfinite(p.box.x)&&std::isfinite(p.box.y)&&
            std::isfinite(p.box.width)&&std::isfinite(p.box.height)&&p.box.width>1&&p.box.height>2&&
            p.box.x>=0&&p.box.y>=0&&p.box.x+p.box.width<=crosswalk.width&&
            item.foot.y<crosswalk.height-1;
        if(!valid_box||!std::isfinite(p.confidence)||p.confidence<.25||!p.run_id||
           p.age_frames!=0||!p.inferred_now||!valid_time(p.source_ms,now,ttl)||
           !valid_time(crosswalk.source_ms,now,ttl)||std::abs(p.source_ms-crosswalk.source_ms)>1.0||
           crosswalk.state==State::Unavailable||crosswalk.state==State::Stale) {
            output.push_back(item);continue;
        }
        if(crosswalk.state==State::NotDetected) item.relation=Relation::NoCrosswalk;
        else if(crosswalk.state==State::Confirmed&&!crosswalk.region.empty()) {
            const double distance=cv::pointPolygonTest(crosswalk.region,item.foot,true);
            const double margin=std::max(2.,crosswalk.height*.012);
            item.relation=distance>margin?Relation::Inside:
                (distance < -margin?Relation::Outside:Relation::Boundary);
        }
        output.push_back(item);
    }
    return output;
}
void draw(cv::Mat& image,const Result& result,const std::vector<PersonAssociation>& persons,bool text) {
    const cv::Scalar color=(result.state==State::Confirmed||result.state==State::Held)?cv::Scalar(255,210,0):cv::Scalar(0,165,255);
    if(!result.region.empty()) {
        // User-selected display band: keep the measured/projected vertical
        // extent and span this camera image. Never rewrite result.region,
        // which remains the evidence used for pedestrian association.
        const cv::Rect bounds=cv::boundingRect(result.region)&cv::Rect(0,0,image.cols,image.rows);
        const std::vector<cv::Point> band=bounds.empty()?std::vector<cv::Point>{}:
            std::vector<cv::Point>{{0,bounds.y},{image.cols-1,bounds.y},
                                  {image.cols-1,bounds.y+bounds.height-1},{0,bounds.y+bounds.height-1}};
        if(result.state==State::Predicted) {
            for(std::size_t i=0;i<band.size();++i) {
                const cv::Point2f a=band[i],b=band[(i+1)%band.size()];
                const double length=cv::norm(b-a);
                for(double d=0;d<length;d+=14)cv::line(image,a+(b-a)*(d/length),a+(b-a)*(std::min(d+8,length)/length),color,2,cv::LINE_8);
            }
        } else if(!band.empty())cv::polylines(image,band,true,color,2,cv::LINE_8);
    }
    for(const auto& p:persons) {
        if(result.state==State::Held)continue;
        const cv::Scalar c=p.relation==Relation::Inside?cv::Scalar(0,0,255):
            (p.relation==Relation::Boundary||p.relation==Relation::Unknown?cv::Scalar(0,165,255):cv::Scalar(255,200,0));
        const auto& b=p.person.box;
        if(!std::isfinite(b.x)||!std::isfinite(b.y)||!std::isfinite(b.width)||
           !std::isfinite(b.height)||b.width<=0||b.height<=0)continue;
        const double x0=std::clamp(static_cast<double>(b.x),0.,static_cast<double>(image.cols));
        const double y0=std::clamp(static_cast<double>(b.y),0.,static_cast<double>(image.rows));
        const double x1=std::clamp(static_cast<double>(b.x)+b.width,0.,static_cast<double>(image.cols));
        const double y1=std::clamp(static_cast<double>(b.y)+b.height,0.,static_cast<double>(image.rows));
        const cv::Rect visible(cv::Point(cvFloor(x0),cvFloor(y0)),cv::Point(cvCeil(x1),cvCeil(y1)));
        if(visible.empty())continue;
        cv::rectangle(image,visible,c,2,cv::LINE_8);
        if(std::isfinite(p.foot.x)&&std::isfinite(p.foot.y)&&p.foot.x>=0&&p.foot.x<image.cols&&p.foot.y>=0&&p.foot.y<image.rows)
            cv::circle(image,p.foot,3,c,cv::FILLED,cv::LINE_8);
        if(text)cv::putText(image,relation_name(p.relation),
            {visible.x,std::max(13,visible.y-5)},cv::FONT_HERSHEY_SIMPLEX,.42,c,1,cv::LINE_AA);
    }
    if(text) {
        const std::string title=std::string("crosswalk band: ")+state_name(result.state)+
            (result.state==State::Predicted?" / age="+std::to_string(cvRound(result.prediction_age_ms))+"ms":" / stripes="+std::to_string(result.stripes.size()));
        const int y=std::max(15,image.rows-10);
        cv::putText(image,title,{8,y},cv::FONT_HERSHEY_SIMPLEX,.5,cv::Scalar(0,0,0),3,cv::LINE_AA);
        cv::putText(image,title,{8,y},cv::FONT_HERSHEY_SIMPLEX,.5,color,1,cv::LINE_AA);
    }
}
const char* state_name(State v){switch(v){case State::Unavailable:return "unavailable";case State::NotDetected:return "not-detected";case State::Candidate:return "candidate";case State::Confirmed:return "confirmed";case State::Stale:return "stale";case State::Held:return "held";case State::Predicted:return "predicted";}return "unavailable";}
const char* relation_name(Relation v){switch(v){case Relation::Unknown:return "unknown";case Relation::NoCrosswalk:return "no-crosswalk";case Relation::Outside:return "outside";case Relation::Boundary:return "boundary";case Relation::Inside:return "inside-crosswalk";}return "unknown";}
}  // namespace p3_crosswalk
