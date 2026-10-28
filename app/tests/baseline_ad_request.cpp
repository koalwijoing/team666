// Real bridge/advisor with a fake clock and heap BAR. No camera or actuator.
#include "p3_decision_bridge.h"
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>

class P3DecisionBridgeAccess {
public:
    static void attach(P3DecisionBridge& b, volatile std::uint8_t* p) { b.bar0_.mapping_=p; b.bar0_.size_=8192; }
    static void detach(P3DecisionBridge& b) { b.bar0_.mapping_=nullptr; b.bar0_.size_=0; }
    static void position(P3DecisionBridge& b, P3LanePosition p) { b.lane_position_=p; }
#ifdef AD_ACK_HELPER
    static unsigned ack(unsigned armed, unsigned cleared, unsigned after) {
        P3DecisionBridge::rearm_acknowledged_events(cleared, after, armed);
        return armed;
    }
#endif
};
static double clock_value=100000.;
static double fake_now() { return clock_value; }
static int checks=0, failures=0;
static void ck(bool ok,const char* name) { ++checks; if(!ok) { ++failures; std::printf("FAIL %s\n",name); } }
static bool turn(const P3DecisionOutput& o) { return o.action==P3Action::TURN_LEFT || o.action==P3Action::TURN_RIGHT; }
static P3RoadSafetyInput road(bool pair=true, P3RoadMarking type=P3RoadMarking::Dashed) {
    P3RoadSafetyInput r;
    r.twinlite_lane.fresh=r.twinlite_lane.valid=pair;
    r.twinlite_lane.left.coef={0,0,180}; r.twinlite_lane.right.coef={0,0,330};
    r.twinlite_lane.left.y_min=r.twinlite_lane.right.y_min=140;
    r.twinlite_lane.left.y_max=r.twinlite_lane.right.y_max=287;
    r.twinlite_lane.left.marking=r.twinlite_lane.right.marking=type;
    r.object_observation_fresh=true;
    return r;
}
static P3Det lamp(const char* s,int cls) {
    P3Det d{}; d.model_id=0; d.cls=cls; d.score=.9f; d.label=s; d.verify_state=kLightVerifyAccepted; return d;
}
struct Fixture {
    std::array<std::uint8_t,8192> bytes{};
    P3DecisionBridge b;
    unsigned frame=0;
    Fixture() { clock_value=100000.; put(0x200,4); put(0x204,4); P3DecisionBridgeAccess::attach(b,bytes.data()); b.set_now_fn_for_test(fake_now); step(0,road(false)); }
    ~Fixture() { P3DecisionBridgeAccess::detach(b); }
    void put(unsigned off,unsigned v) { std::memcpy(bytes.data()+off,&v,4); }
    unsigned get(unsigned off) { unsigned v; std::memcpy(&v,bytes.data()+off,4); return v; }
    P3DecisionOutput step(unsigned events=0,P3RoadSafetyInput r=road(),double dt=100.,
                          std::vector<P3Det> det={},unsigned bad_abi=0,double age=20.) {
        clock_value+=dt; ++frame;
        put(0x204,get(0x200)|bad_abi); put(0x208,events); put(0x20c,0);
        r.decision_now_ms=clock_value;
        if(r.twinlite_lane.fresh) {
            r.twinlite_lane.source_ms=clock_value-age;
            r.twinlite_lane.source_run_id=frame; r.twinlite_lane.source_eye=0;
        }
        if(r.object_observation_fresh) { r.object_observation_source_ms=clock_value-age; r.object_observation_run_id=frame; }
        for(auto& o:r.objects) o.source_ms=clock_value-age;
        P3LampObservation l; l.status=1; l.run_id=frame; l.capture_done_ms=clock_value-10;
        return b.update(det,frame,&l,&r);
    }
};
int main() {
    for(unsigned ev:{4u,8u}) {
        Fixture f; auto o=f.step(ev,road(false));
        ck(!turn(o) && o.lane==P3LanePosition::STRAIGHT,"missing pair cannot grant immediately");
        ck(!o.alarm && o.reason.find("waiting:")!=std::string::npos,"transient missing pair is acknowledged waiting, not alarm");
        o=f.step(); ck(o.action==(ev==4?P3Action::TURN_LEFT:P3Action::TURN_RIGHT),"one press executes when next fresh evidence arrives");
        ck(o.reason.find("granted")!=std::string::npos,"delayed grant has visible final receipt");
        o=f.step(); ck(!turn(o),"grant executes once, no stale turn permit is held");
    }
    { Fixture f; f.step(4,road(false)); auto o=f.step(0,road(),1000.);
      ck(!turn(o) && o.reason.find("expired:")!=std::string::npos,"fresh evidence at deadline cannot revive expired request");
      ck(!turn(f.step()),"expiry requires a new physical request"); }
    { Fixture f; f.step(4,road(false)); f.step(0,road(false),600.); f.step(4,road(false),200.);
      auto o=f.step(0,road(),201.); ck(!turn(o),"repeated same-side key never extends original deadline"); }
    for(auto mark:{P3RoadMarking::Solid,P3RoadMarking::Conflict}) {
        Fixture f; f.step(4,road(false)); auto o=f.step(0,road(true,mark));
        ck(!turn(o) && o.alarm,"solid or conflict cancels pending immediately");
        ck(!turn(f.step()),"known marking refusal cannot auto-retry later");
    }
    { Fixture f; f.step(4,road(false)); auto r=road(); r.objects={{80,150,240,340,.9f,2,0,0}};
      auto o=f.step(0,r); ck(!turn(o) && o.alarm,"near left neighbor cancels left request");
      ck(!turn(f.step()),"neighbor going away does not revive denied request"); }
    { Fixture f; f.step(8,road(false)); auto r=road(); r.objects={{130,150,285,340,.9f,2,0,0}};
      auto o=f.step(0,r); ck(!turn(o) && o.alarm,"unlocalized boundary risk never counts as clear");
      ck(!turn(f.step()),"unresolved object refusal is final for that request"); }
    { Fixture f; f.step(4,road(false)); auto r=road(); r.objects={{250,180,370,340,.9f,2,0,0}};
      auto o=f.step(0,r); ck(o.action==P3Action::STOP && o.lane==P3LanePosition::STRAIGHT,"front stop overrides pending without lane commit");
      ck(!turn(f.step()),"front safety veto consumes pending request"); }
    for(unsigned key:{1u,2u,16u,12u}) {
        Fixture f; f.step(4,road(false)); auto o=f.step(key,road(false));
        ck(!turn(o),"S W F or A+D cancels pending");
        o=f.step(); ck(!turn(o),"cancelled request cannot later execute");
    }
    { Fixture f; f.step(4,road(false)); auto o=f.step(8);
      ck(o.action==P3Action::TURN_RIGHT,"opposite key replaces waiting intent, never executes old direction");
      ck(!turn(f.step()),"opposite replacement executes once"); }
    { Fixture f; f.step(4,road(false)); auto o=f.step(0,road(),100.,{},0x40);
      ck(o.action==P3Action::STOP && o.alarm,"ABI fault stops and cancels pending");
      ck(!turn(f.step()),"ABI recovery cannot execute old request"); }
    { Fixture f; f.step(4,road(false)); f.b.reset_state(); ck(!turn(f.step()),"warmup/reset clears pending"); }
    { Fixture f; f.step(4,road(false)); auto o=f.step(0,road(),100.,{lamp("stop",3)});
      ck(o.action==P3Action::STOP && !turn(o),"new red immediately cancels pending and stops");
      for(int i=0;i<40;++i)o=f.step(); ck(!turn(o),"red clearing later never revives intent"); }
    for(auto pos:{P3LanePosition::LEFT,P3LanePosition::RIGHT}) {
        Fixture f; P3DecisionBridgeAccess::position(f.b,pos); auto o=f.step(pos==P3LanePosition::LEFT?8:4);
        ck(turn(o) && o.lane==P3LanePosition::STRAIGHT,"one A/D press moves one lane, never skips straight lane");
    }
    { Fixture f; P3DecisionBridgeAccess::position(f.b,P3LanePosition::LEFT); auto o=f.step(4);
      ck(!turn(o) && o.alarm,"leftmost boundary still rejects A"); }
    { Fixture f; f.step(4,road(false)); auto o=f.step(0,road(),100.,{},0,401.);
      ck(!turn(o),"pending does not make 401ms source fresh"); }
    { Fixture f; auto r=road(); r.object_observation_fresh=false;
      auto o=f.step(8,r); ck(!turn(o) && o.reason.find("waiting:")!=std::string::npos,"missing object observation waits, not fabricated clear");
      o=f.step(); ck(o.action==P3Action::TURN_RIGHT,"fresh successful empty observation can satisfy pending"); }
    { Fixture f; f.step(4,road(false)); clock_value-=1000.; auto o=f.step();
      ck(!turn(o),"backward clock cancels pending"); }
#ifdef AD_ACK_HELPER
    ck(P3DecisionBridgeAccess::ack(0,4,0)==4,"successful W1C readback re-arms cleared A immediately");
    ck(P3DecisionBridgeAccess::ack(0,4,4)==0,"stuck-high A is not rearmed");
    ck(P3DecisionBridgeAccess::ack(16,12,8)==20,"only acknowledged low bits rearm, unrelated bits preserved");
    ck(P3DecisionBridgeAccess::ack(0,4,0xffffffe0)==4,"reserved bits cannot fabricate armed keys");
#endif
    std::printf("AD_REQUEST_TESTS checks=%d failures=%d\n",checks,failures);
    return failures?1:0;
}
