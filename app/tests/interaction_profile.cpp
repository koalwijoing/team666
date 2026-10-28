// Reuse the production bridge/heap-BAR fixture, not an imitation gate.
#define main original_regression_main
#include "baseline_ad_request.cpp"
#undef main
#include <cstdlib>

int main()
{
    setenv("P3_AD_RETRY_UNCERTAIN", "1", 1);
    setenv("P3_AD_REQUEST_WAIT_MS", "2000", 1);
    setenv("P3_SIM_LANE_EDGE_STRICT", "0", 1);
    checks = failures = 0;
    for (unsigned key : {4u, 8u}) {
        Fixture f;
        auto o=f.step(key,road(false));
        ck(!turn(o),"missing pair never grants");
        o=f.step(0,road(),1500.);
        ck(o.action==(key==4?P3Action::TURN_LEFT:P3Action::TURN_RIGHT),"one press survives 1.5s evidence gap");
        ck(!turn(f.step()),"each request still commits once");
    }
    { Fixture f;f.step(4,road(false));auto o=f.step(0,road(),2000.);
      ck(!turn(o)&&o.reason.find("expired:")!=std::string::npos,"2s deadline is absolute");
      ck(!turn(f.step()),"expired request cannot revive"); }
    { Fixture f;f.step(4,road(false));f.step(4,road(false),1500.);auto o=f.step(0,road(),501.);
      ck(!turn(o),"same-key repeat cannot extend deadline"); }
    for(unsigned key:{4u,8u}) {
        Fixture f;P3DecisionBridgeAccess::position(f.b,key==4?P3LanePosition::LEFT:P3LanePosition::RIGHT);
        auto o=f.step(key);
        ck(turn(o),"fresh dashed target is not blocked by assumed ordinal edge");
        ck(o.lane==(key==4?P3LanePosition::LEFT:P3LanePosition::RIGHT),"display position stays in enum range");
        f.step(); // Physical release/low read re-arms the fake W1C event.
        o=f.step(key);
        ck(turn(o),"another fresh physical key can request the same visible dashed crossing");
        f.step();
        o=f.step(key,road(true,P3RoadMarking::Solid));
        ck(!turn(o)&&o.alarm,"ordinal bypass never bypasses a solid line");
        ck(!turn(f.step()),"solid refusal still consumes intent");
    }
    { Fixture f;auto r=road();r.objects={{130,150,285,340,.9f,2,0,0}};
      auto o=f.step(8,r);
      ck(!turn(o)&&o.action==P3Action::SLOW,"unresolved boundary target keeps caution and forbids turn");
      ck(o.reason.find("waiting:")!=std::string::npos,"uncertain blocker gives waiting feedback");
      o=f.step();ck(o.action==P3Action::TURN_RIGHT,"fresh proven clearance can satisfy waiting intent");
      ck(!turn(f.step()),"recovered intent executes once"); }
    { Fixture f;auto r=road();r.objects={{130,20,200,50,.9f,2,0,0}};
      auto o=f.step(8,r);ck(!turn(o),"far unlocalized object is never fabricated clear");
      ck(o.reason.find("waiting:")!=std::string::npos,"far unlocalized evidence waits instead of consuming key");
      o=f.step();ck(o.action==P3Action::TURN_RIGHT,"fresh resolution clears far uncertain target"); }
    for(unsigned key:{4u,8u}) {
      Fixture f;auto r=road();r.objects=key==4?std::vector<P3RoadObject>{{70,150,210,340,.9f,2,0,0}}:
                                                   std::vector<P3RoadObject>{{440,150,580,340,.9f,2,0,0}};
      auto o=f.step(key,r);ck(!turn(o)&&o.alarm,"near target-side road user still blocks");
      ck(!turn(f.step()),"known neighbor refusal cannot revive");
    }
    for(auto mark:{P3RoadMarking::Solid,P3RoadMarking::Conflict}){
      Fixture f;auto o=f.step(4,road(true,mark));ck(!turn(o)&&o.alarm,"solid and conflicting marking still block");
      ck(!turn(f.step()),"concrete marking refusal still consumes key");
    }
    for(const auto& box:std::vector<P3RoadObject>{{250,180,370,340,.9f,2,0,0},{250,270,370,340,.9f,2,0,0}}){
      Fixture f;auto r=road();r.objects={box};auto o=f.step(4,r);
      ck(!turn(o)&&(o.action==P3Action::STOP||o.action==P3Action::SLOW),"real front STOP/SLOW keeps priority");
      ck(!turn(f.step()),"real front veto consumes key");
    }
    { Fixture f;auto o=f.step(4,road(),100.,{lamp("stop",3)});
      ck(o.action==P3Action::STOP&&!turn(o),"red still stops immediately");
      for(int i=0;i<40;++i)o=f.step();ck(!turn(o),"red clearing cannot revive key"); }
    { Fixture f;for(int i=0;i<7;++i)f.step(0,road(),100.,{lamp("warning",5)});
      auto o=f.step(4,road(),100.,{lamp("warning",5)});
      ck(!turn(o),"yellow never authorizes turn");ck(!turn(f.step()),"yellow consumes key"); }
    { Fixture f;auto o=f.step(4,road(),100.,{},0,401.);
      ck(!turn(o),"2s request never extends 400ms perception TTL");
      o=f.step();ck(o.action==P3Action::TURN_LEFT,"new fresh observation can satisfy request"); }
    for(unsigned key:{1u,2u,16u,12u}){
      Fixture f;f.step(4,road(false));auto o=f.step(key,road(false));ck(!turn(o),"S/W/F/chord cancels pending");
      ck(!turn(f.step()),"control cancellation never revives");
    }
    { Fixture f;f.step(4,road(false));auto o=f.step(0,road(),100.,{},0x40);
      ck(o.action==P3Action::STOP&&!turn(o),"ABI failure still stops");
      ck(!turn(f.step()),"ABI recovery cannot revive key"); }
    std::printf("INTERACTION_PROFILE checks=%d failures=%d\n",checks,failures);
    return failures?1:0;
}
