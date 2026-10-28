#include "crosswalk_person.h"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

int checks=0;
void require(bool value,const char* name){++checks;if(!value)throw std::runtime_error(name);}
cv::Mat crosswalk_image(){
    cv::Mat image(360,640,CV_8UC3,cv::Scalar(65,65,65));
    for(int mid=100;mid<=550;mid+=75){
        const int far=320+(mid-320)/2;
        std::vector<cv::Point> q{{far-9,235},{far+9,235},{mid+17,325},{mid-17,325}};
        cv::fillConvexPoly(image,q,{225,225,225});
    }
    return image;
}
int main(){try{
    using namespace p3_crosswalk;
    const cv::Mat positive=crosswalk_image();
    Detector detector;
    auto r=detector.update({1,1000,1001,positive,{}});
    require(r.state==State::Candidate,"single observation only candidate");
    require(r.stripes.size()>=4,"multiple perspective stripes found");
    require(detector.update({1,1000,1100,positive,{}}).confirmations==1,"cached observation cannot add vote");
    require(detector.update({2,1000,1100,positive,{}}).confirmations==1,"aliased capture time cannot add vote");
    r=detector.update({3,1110,1111,positive,{}});
    require(r.state==State::Confirmed,"distinct coherent observations confirm");
    require(r.source_ms==1110,"capture source preserved");
    const auto held=detector.view(4,1120);
    require(held.state==State::Held&&held.source_ms==1110&&held.delivery_id==3,"held display preserves source identity and clock");
    require(detector.view(3,1511).state==State::Stale,"expired observation stale");
    require(detector.view(6,1112).region.empty(),"old frame geometry hidden");
    require(detector.view(2,1112).state==State::Stale,"future delivery rejected");
    require(detector.view(3,1000).state==State::Stale,"future timestamp rejected");
    cv::Mat blank(360,640,CV_8UC3,cv::Scalar(65,65,65));
    require(detector.update({4,1120,1121,blank,{}}).state==State::NotDetected,"new negative clears old region");
    require(detector.update({5,1130,1131,{}, {}}).state==State::Unavailable,"empty frame unavailable");
    require(detector.update({6,2000,1131,positive,{}}).state==State::Unavailable,"invalid source clock unavailable");
    cv::Mat mono(360,640,CV_8U,cv::Scalar(0));
    require(detector.update({7,1140,1141,mono,{}}).state==State::Unavailable,"wrong image type rejected");
    cv::Mat stop=blank.clone();cv::rectangle(stop,{20,280,600,14},{230,230,230},cv::FILLED);
    require(detector.update({8,1150,1151,stop,{}}).state==State::NotDetected,"one stop line not a crosswalk");
    cv::Mat lanes=blank.clone();cv::line(lanes,{90,350},{280,190},{235,235,235},8);cv::line(lanes,{560,350},{360,190},{235,235,235},8);
    require(detector.update({9,1160,1161,lanes,{}}).state==State::NotDetected,"lane boundaries not a crosswalk");
    cv::Mat cars=blank.clone();for(int x=15;x<610;x+=145)cv::rectangle(cars,{x,250,95,28},{230,230,230},cv::FILLED);
    const auto roofs=detector.update({10,1170,1171,cars,{}});
    if(roofs.state!=State::NotDetected)for(const auto& stripe:roofs.stripes)
        std::cerr<<"roof stripe "<<cv::boundingRect(stripe)<<"\n";
    require(roofs.state==State::NotDetected,"horizontal vehicle roofs rejected");
    cv::Mat arrows=blank.clone();for(int x=140;x<600;x+=180)cv::arrowedLine(arrows,{x,330},{x,230},{230,230,230},10,cv::LINE_8,0,.35);
    require(detector.update({11,1180,1181,arrows,{}}).state==State::NotDetected,"road arrows not a crosswalk");
    cv::Mat bands=blank.clone();
    cv::rectangle(bands,{8,260,200,12},{225,225,225},cv::FILLED);
    cv::rectangle(bands,{8,285,250,12},{225,225,225},cv::FILLED);
    cv::rectangle(bands,{8,310,300,12},{225,225,225},cv::FILLED);
    cv::Mat band_mask;cv::cvtColor(bands,band_mask,cv::COLOR_BGR2GRAY);cv::threshold(band_mask,band_mask,150,255,cv::THRESH_BINARY);
    require(detector.update({12,1190,1191,bands,{}}).state==State::NotDetected,"parallel bank requires marking mask support");
    require(detector.update({13,1200,1201,bands,band_mask}).state==State::Candidate,"masked oblique band bank recognized");
    require(detector.update({14,1210,1211,bands,band_mask}).state==State::Confirmed,"distinct masked bank observations confirm");
    cv::Mat roof_mask;cv::cvtColor(cars,roof_mask,cv::COLOR_BGR2GRAY);cv::threshold(roof_mask,roof_mask,150,255,cv::THRESH_BINARY);
    require(detector.update({15,1220,1221,cars,roof_mask}).state==State::NotDetected,"even masked vehicle roofs fail thin-bank geometry");
    cv::Mat slanted=blank.clone();
    for(const auto& q:std::vector<std::vector<cv::Point>>{
        {{8,268},{130,258},{130,266},{8,276}},
        {{8,290},{186,284},{186,292},{8,298}},
        {{8,314},{286,306},{286,314},{8,322}}})cv::fillConvexPoly(slanted,q,{225,225,225});
    cv::Mat slanted_mask;cv::cvtColor(slanted,slanted_mask,cv::COLOR_BGR2GRAY);cv::threshold(slanted_mask,slanted_mask,150,255,cv::THRESH_BINARY);
    require(detector.update({16,1230,1231,slanted,slanted_mask}).state==State::Candidate,"shared-column contrast follows oblique stripe axes");
    cv::Mat horizon=blank.clone();
    for(int mid=100;mid<=550;mid+=75){
        const int far=320+(mid-320)/2;
        cv::fillConvexPoly(horizon,std::vector<cv::Point>{{far-9,178},{far+9,178},{mid+17,207},{mid-17,207}},{225,225,225});
    }
    require(detector.update({17,1240,1241,horizon,{}}).state==State::NotDetected,"repeated bright horizon facades outside road band rejected");
    detector.reset();require(detector.view(1,1200).state==State::Unavailable,"reset clears state");

    Result region;region.state=State::Confirmed;region.source_ms=1000;region.delivery_id=10;region.width=640;region.height=360;
    region.region={{100,240},{500,240},{500,330},{100,330}};
    Person p{{200,200,40,90},.8f,8,1000,0,true};
    require(associate(region,{p},1100)[0].relation==Relation::Inside,"foot inside region");
    p.box.x=50;require(associate(region,{p},1100)[0].relation==Relation::Outside,"sidewalk outside region");
    p.box.x=80;require(associate(region,{p},1100)[0].relation==Relation::Boundary,"edge kept uncertain");
    p.box.x=200;p.source_ms=990;require(associate(region,{p},1100)[0].relation==Relation::Unknown,"different source frames not associated");
    p.source_ms=1000;p.age_frames=1;require(associate(region,{p},1100)[0].relation==Relation::Unknown,"cached box not current position");
    p.age_frames=0;p.inferred_now=false;require(associate(region,{p},1100)[0].relation==Relation::Unknown,"tracked box kept uncertain");
    p.inferred_now=true;p.run_id=0;require(associate(region,{p},1100)[0].relation==Relation::Unknown,"missing run identity rejected");
    p.run_id=8;require(associate(region,{p},1401)[0].relation==Relation::Unknown,"expired person and region rejected");
    p.box.height=160;require(associate(region,{p},1100)[0].relation==Relation::Unknown,"truncated feet not localized");
    p.box.height=90;region.state=State::Candidate;require(associate(region,{p},1100)[0].relation==Relation::Unknown,"candidate cannot confirm person association");
    region.state=State::NotDetected;require(associate(region,{p},1100)[0].relation==Relation::NoCrosswalk,"available negative distinct from unavailable");
    region.state=State::Unavailable;require(associate(region,{p},1100)[0].relation==Relation::Unknown,"missing observation not a negative");
    region.state=State::Held;require(associate(region,{p},1100)[0].relation==Relation::Unknown,"held region cannot assert current person position");
    region.state=State::Unavailable;
    p.box.x=std::numeric_limits<float>::quiet_NaN();auto invalid=associate(region,{p},1100);
    require(invalid[0].relation==Relation::Unknown,"nonfinite box rejected");
    cv::Mat rendered=blank.clone();draw(rendered,region,invalid);
    require(cv::norm(rendered,blank,cv::NORM_L1)>0,"invalid box rendering remains bounded and status visible");
    bool threw=false;try{Detector bad({100,4,400,2,2});}catch(const std::invalid_argument&){threw=true;}
    require(threw,"invalid analysis size rejected");
    PredictiveDetector tracked;
    auto tr=tracked.update({1,1000,1001,positive,{}});
    require(tr.state==State::Candidate,"prediction cannot start from a single candidate");
    require(tracked.view(2,1051,1050).state!=State::Predicted,"unconfirmed candidate never seeds prediction");
    tr=tracked.update({2,1100,1101,positive,{}});
    require(tr.state==State::Confirmed,"two real observations establish region track");
    cv::Mat shifted;
    cv::warpAffine(positive,shifted,cv::Matx23d(1,0,8,0,1,0),positive.size(),cv::INTER_NEAREST,cv::BORDER_CONSTANT,{65,65,65});
    tr=tracked.update({3,1200,1201,shifted,{}});
    require(tr.state==State::Confirmed,"coherent moving geometry corrects filter");
    const auto measured_bounds=cv::boundingRect(tr.region);
    const auto between=tracked.view(4,1251,1250);
    require(between.state==State::Predicted,"intermediate image gets explicitly predicted region");
    require(between.delivery_id==3&&between.source_ms==1200&&between.projection_ms==1250,"prediction preserves measurement identity and labels projection time");
    require(between.stripes.empty()&&between.geometry_score==0,"prediction never fabricates observed stripes or score");
    tr=tracked.update({4,1300,1301,blank,{}});
    require(tr.state==State::Predicted,"short geometry dropout uses Kalman forecast");
    require(cv::boundingRect(tr.region).x>measured_bounds.x,"forecast follows learned movement instead of freezing polygon");
    require(tr.source_ms==1200&&tr.prediction_age_ms==100,"failed new image cannot refresh actual measurement clock");
    const auto duplicate=tracked.update({4,1300,1301,blank,{}});
    require(duplicate.region==tr.region&&duplicate.prediction_sigma_px==tr.prediction_sigma_px,"duplicate delivery cannot advance filter");
    Person forecast_person{{220,200,40,90},.8f,4,1300,0,true};
    require(associate(tr,{forecast_person},1301)[0].relation==Relation::Unknown,"predicted region cannot assert a measured pedestrian relation");
    const double sigma=tr.prediction_sigma_px;
    tr=tracked.update({5,1400,1401,blank,{}});
    require(tr.state==State::Predicted&&tr.prediction_sigma_px>sigma,"model uncertainty grows through dropout");
    require(tracked.view(4,1402,1400).state==State::Stale,"out-of-order display rejected");
    require(tracked.view(6,1402,1400).state==State::Stale,"aliased source image cannot be projected as a new capture");
    require(tracked.view(9,1601,1600).region.empty(),"prediction expires by wall age even when image age is at horizon");
    tr=tracked.update({6,1700,1701,blank,{}});
    require(tr.region.empty()&&tr.state==State::NotDetected,"long dropout withdraws region");
    tr=tracked.update({7,1800,1801,positive,{}});
    require(tr.state==State::Candidate,"expired track needs fresh confirmation");
    tracked.update({8,1900,1901,positive,{}});
    tr=tracked.update({9,2000,2001,cv::Mat(360,640,CV_8UC3,cv::Scalar(220,220,220)),{}});
    require(tr.region.empty()&&tr.state!=State::Predicted,"large scene appearance cut clears old track");
    tracked.reset();tracked.update({1,1000,1001,positive,{}});tracked.update({2,1100,1101,positive,{}});
    tr=tracked.update({3,1100,1101,positive,{}});
    require(tr.state==State::Unavailable,"aliased capture time resets predictive state");
    tracked.update({4,1200,1201,positive,{}});tracked.update({5,1300,1301,positive,{}});
    tr=tracked.update({6,1400,1401,{}, {}});
    require(tr.state==State::Unavailable&&tracked.view(7,1451,1450).region.empty(),"invalid image never coasts old track");
    tracked.reset();tracked.update({1,1000,1001,positive,{}});tracked.update({2,1100,1101,positive,{}});
    cv::Mat jumped;cv::warpAffine(positive,jumped,cv::Matx23d(1,0,150,0,1,0),positive.size(),cv::INTER_NEAREST,cv::BORDER_CONSTANT,{65,65,65});
    tr=tracked.update({3,1200,1201,jumped,{}});
    require(tr.state==State::Candidate,"large innovation starts a new candidate instead of joining tracks");
    Config disabled;disabled.prediction_enabled=false;PredictiveDetector plain(disabled);
    plain.update({1,1000,1001,positive,{}});plain.update({2,1100,1101,positive,{}});
    require(plain.update({3,1200,1201,blank,{}}).state==State::NotDetected,"prediction switch preserves baseline dropout behavior");
    cv::Mat dashed=blank.clone();draw(dashed,between,{});
    require(cv::norm(dashed,blank,cv::NORM_L1)>0,"predicted region renders with distinct status");
    std::cout<<"CROSSWALK_TESTS checks="<<checks<<" failures=0\n";return 0;
}catch(const std::exception& e){std::cerr<<"check "<<checks<<": "<<e.what()<<"\n";return 1;}}
