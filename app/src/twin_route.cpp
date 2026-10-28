#include <string>
#include <cstdlib>
#include <cstdio>
#include <array>
#include <algorithm>
extern "C" bool __real__ZN17TwinliteLaneModel4initERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE(void*,const std::string&);
extern "C" bool __wrap__ZN17TwinliteLaneModel4initERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE(void* self,const std::string& path){
 const std::array<std::string,4> allowed={
  "/root/board_deploy/twinlite_small_lane_288x512_int8_calib128.rknn",
  "/userdata/codex_rknn_accept_20261026_r01/models/twin_pad.rknn",
  "/userdata/codex_rknn_accept_20261026_r01/models/twin_caam.rknn",
  "/userdata/codex_rknn_accept_20261026_r01/models/twin_both.rknn"};
 const char* env=std::getenv("P3_TEST_TWIN_MODEL");const std::string selected=env&&*env?env:allowed[0];
 if(path!=allowed[0] || std::find(allowed.begin(),allowed.end(),selected)==allowed.end()){
  std::fprintf(stderr,"TWIN_ROUTE_REJECT\n");return false;}
 std::printf("TWIN_MODEL_ROUTE source=%s selected=%s\n",path.c_str(),selected.c_str());
 return __real__ZN17TwinliteLaneModel4initERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE(self,selected);
}
