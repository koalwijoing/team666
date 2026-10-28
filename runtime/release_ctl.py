"""Publish the user-accepted simulated lane interaction; retain exact rollback."""
from pathlib import Path
import fcntl,hashlib,json,mmap,os,re,shutil,struct,subprocess,sys,time,traceback
R=Path('/userdata/codex_lane_interaction_release_20261028_r01')
C=Path('/userdata/codex_lane_request_wait_20261028_r01')
sys.path.insert(0,str(C/'scripts'));from guard import atomic,identity,run
ENTRY=Path('/userdata/claude_qt_async_default_20261021/start_qt_async_default.sh')
OLD='/userdata/codex_pad_crosswalk_release_20261027_r01/build/app';APP=str(R/'build/app')
KEY='/userdata/claude_four_capture_20261023/keyinject_ctl.sh'
GUI='/userdata/codex_no_speed_caam_pad_20261026_r01/build/gui_quit'
MODEL='/userdata/codex_rknn_accept_20261026_r01/models/twin_pad.rknn'
OLD_ENTRY_SHA='f7bd486c43f498fb4ba75cc005304c4092297cea6febee34a27fafbc411ef2ab'
APP_SHA='3b1d686cb900673a92a36d9c6038da65c37c2c653e43cccb01784851319a3afd'
ENV=dict(os.environ,DISPLAY=':0',XAUTHORITY='/home/linaro/.Xauthority')
PUB=R/'publication_r01';OUT=PUB;STATE=OUT/'state.json'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def owners():
 result=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdecimal():continue
  try:
   if any('pango_pci_driver' in os.readlink(f) for f in (p/'fd').iterdir()):result.append(identity(int(p.name)))
  except OSError:pass
 return [x for x in result if x]
def latest():return Path(Path('/userdata/claude_qt_async_default_20261021/latest_run.txt').read_text().strip())/'app.log'
def no_capture():
 assert not owners(),'capture FD remains'
 for p in Path('/proc').iterdir():
  if not p.name.isdecimal():continue
  try:assert 'pango_pci_driver' not in (p/'maps').read_text(),'capture mapping remains'
  except OSError:pass
def replace(data):
 tmp=ENTRY.with_name(ENTRY.name+'.lane_interaction_tmp');assert not tmp.exists()
 with tmp.open('xb') as f:f.write(data);f.flush();os.fsync(f.fileno())
 tmp.chmod(0o755);os.replace(tmp,ENTRY)
def mark(**kw):
 s=json.loads(STATE.read_text());s.update(kw);atomic(STATE,s)
def stop_current(label):
 run(['bash',KEY,'stop'],OUT/(label+'_keys.log'),12)
 own=owners()
 if not own:
  # A clean completed preview can have no capture owner. Do not invent DMA proof.
  assert not json.loads(STATE.read_text()).get('capture_unconfirmed');no_capture();return
 assert len(own)==1 and own[0]['exe'] in (APP,OLD)
 who=own[0];log=latest();mark(capture_unconfirmed=True,stopping=who)
 for attempt in range(10):
  current=identity(who['pid'])
  if not current:break
  assert current['start']==who['start'] and current['exe']==who['exe']
  try:run([GUI,who['pid'],'stop'],OUT/(label+'_gui_'+str(attempt)+'.log'),8,env=ENV);break
  except RuntimeError:time.sleep(1)
 else:raise RuntimeError('normal GUI exit could not be requested')
 deadline=time.monotonic()+30
 while identity(who['pid']) and time.monotonic()<deadline:time.sleep(.15)
 assert not identity(who['pid']),'normal process exit not confirmed'
 assert 'compute_exit=0 gui_exit_confirmed=1' in log.read_text(errors='replace')[-10000:]
 no_capture();mark(capture_unconfirmed=False)
def key_start(label):
 with (OUT/(label+'.log')).open('xb') as f:
  p=subprocess.Popen(['bash',KEY,'start'],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
  assert p.wait(timeout=12)==0
 pid=int(Path('/userdata/claude_keyinject_20261021/keyinject.pid').read_text());who=identity(pid)
 assert who and who['state'] not in 'TXZ'
 assert b'/userdata/claude_keyinject_20261021/keyinject.py' in (Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0')
 return who
def progress(expected,profile,label):
 deadline=time.monotonic()+60;stable=None
 while time.monotonic()<deadline:
  own=owners()
  if len(own)!=1 or own[0]['exe']!=expected:time.sleep(.5);continue
  text=latest().read_text(errors='replace');frames=re.findall(r'\[twinlite\] frame=(\d+)',text)
  gui=re.search(r'\[gui-main\] completed=\d+',text)
  if not frames or int(frames[-1])<60 or not gui:time.sleep(.5);continue
  assert 'selected='+MODEL in text and '[crosswalk] enabled=1' in text
  assert not re.search(r'fail=[1-9]|FATAL|TWIN_ROUTE_REJECT|model init failed',text)
  who=own[0];env=dict(x.decode().split('=',1) for x in (Path('/proc')/str(who['pid'])/'environ').read_bytes().split(b'\0') if b'=' in x)
  required={'P3_TEST_TWIN_MODEL':MODEL,'P3_ENABLE_CROSSWALK':'1','P3_ENABLE_CROSSWALK_PREDICTION':'1',
            'P3_ENABLE_SPEED':'0','P3_ENABLE_SIGN':'0','P3_LIGHT_INTERVAL':'2','P3_LANE_INTERVAL':'2','P3_OBJECT_INTERVAL':'1'}
  if profile:required.update(P3_AD_REQUEST_WAIT_MS='2000',P3_AD_RETRY_UNCERTAIN='1',P3_SIM_LANE_EDGE_STRICT='0')
  for k,v in required.items():assert env[k]==v,(k,env.get(k))
  first=int(frames[-1]);time.sleep(3);now=owners()
  if len(now)!=1 or now[0]['pid']!=who['pid'] or now[0]['start']!=who['start']:continue
  frames=re.findall(r'\[twinlite\] frame=(\d+)',latest().read_text(errors='replace'))
  if int(frames[-1])<=first:continue
  stable={'owner':now[0],'frame_first':first,'frame_last':int(frames[-1]),'environment':{k:env[k] for k in required},'hw_osd':env['P3_HW_OSD'],'log':str(latest())};break
 assert stable,'stable GUI/frame advance not confirmed'
 stable['keyboard']=key_start(label+'_keyboard');return stable
def start(expected,profile,label):
 assert not json.loads(STATE.read_text()).get('capture_unconfirmed');no_capture()
 run(['bash',str(ENTRY),'live'],OUT/(label+'_entry.log'),25,env=ENV)
 return progress(expected,profile,label)
def rollback(label):
 plan=json.loads((PUB/'plan.json').read_text());assert sha(ENTRY) in (plan['old_entry_sha256'],plan['new_entry_sha256'])
 stop_current(label);replace((PUB/'entry_before.sh').read_bytes())
 result=start(OLD,False,label+'_old');atomic(OUT/(label+'_restoration.json'),result);return result
def watch():
 os.setsid()
 while True:
  time.sleep(.5);s=json.loads(STATE.read_text())
  if s['complete']:return
  p=identity(s['parent']['pid'])
  if not p or p['start']!=s['parent']['start'] or p['state']=='Z' or time.monotonic()>s['deadline']:break
 try:atomic(OUT/'watchdog_rollback.json',rollback('watchdog'))
 except BaseException:atomic(OUT/'watchdog_failed.json',{'error':traceback.format_exc()})
def publish():
 assert not R.exists();R.mkdir();(R/'build').mkdir();(R/'scripts').mkdir();PUB.mkdir()
 assert sha(C/'build/app')==APP_SHA and sha(ENTRY)==OLD_ENTRY_SHA
 assert sha(OLD)=='eff83ff7af14c869853fae9974cc7fcb6e197c124a671ec4fad5c46c92416825'
 assert sha(MODEL)=='938040480f56a93c31447ea303abb7860cb45740b716968c3ec66f511d4ec0a0'
 assert json.loads((C/'live_preview_r03/state.json').read_text())['complete']
 preview=json.loads((C/'live_preview_r03/receipt.json').read_text());assert preview['ready'] and preview['normal_exit'] and preview['restoration']['formal_resumed']
 shutil.copy2(C/'build/app',R/'build/app');assert sha(R/'build/app')==APP_SHA
 # The current script is installed by the caller before any publication mutation.
 shutil.copy2(C/'scripts/release_ctl.py',R/'scripts/release_ctl.py')
 before=ENTRY.read_bytes();text=before.decode();assert text.count('APP='+OLD)==1
 text=text.replace('APP='+OLD,'APP='+APP,1).replace('APP_SHA=eff83ff7af14c869853fae9974cc7fcb6e197c124a671ec4fad5c46c92416825','APP_SHA='+APP_SHA,1)
 anchor='export P3_TEST_TWIN_MODEL="$TWIN_PAD_MODEL"\n';assert text.count(anchor)==1
 text=text.replace(anchor,anchor+'# User-accepted simulated lane interaction, 2026-10-28.\nexport P3_AD_REQUEST_WAIT_MS=2000 P3_AD_RETRY_UNCERTAIN=1 P3_SIM_LANE_EDGE_STRICT=0\n',1)
 anchor='--summary-csv "$OUT/summary.csv" --perf-summary "$OUT/perf_summary.csv"'
 assert text.count(anchor)==1;text=text.replace(anchor,'--raw-csv "$OUT/experiment_frames.csv" --perf-frames "$OUT/perf_frames.csv"\n          '+anchor,1)
 # Stop/rollback use clean GUI shutdown rather than terminating an in-flight DMA.
 begin=text.index('  stop)\n');end=text.index('  *)\n',begin)
 text=text[:begin]+'''  stop)
    exec 9>&-
    exec python3 /userdata/codex_lane_interaction_release_20261028_r01/scripts/release_ctl.py stop ;;
  rollback)
    exec 9>&-
    exec python3 /userdata/codex_lane_interaction_release_20261028_r01/scripts/release_ctl.py rollback ;;
'''+text[end:]
 after=text.encode();(PUB/'entry_before.sh').write_bytes(before);(PUB/'entry_after.sh').write_bytes(after)
 plan={'old_entry_sha256':sha(ENTRY),'new_entry_sha256':hashlib.sha256(after).hexdigest(),'app_sha256':APP_SHA,'app':APP,
       'previous_app':OLD,'model':MODEL,'desktop_unchanged':True,'fpga_changed':False,'default_profile':{'wait_ms':2000,'retry_uncertain':True,'strict_simulated_edge':False},
       'historical_csv_preserved':True,'new_csv_files_per_run':True,'preview_stats':preview['stats']}
 atomic(PUB/'plan.json',plan);run(['bash','-n',PUB/'entry_after.sh'],PUB/'syntax.log',10)
 atomic(STATE,{'parent':identity(os.getpid()),'deadline':time.monotonic()+180,'complete':False,'capture_unconfirmed':False})
 watcher=os.fork()
 if watcher==0:
  try:watch()
  finally:os._exit(0)
 result=dict(plan)
 try:
  stop_current('old_stop');replace(after);result['running']=start(APP,True,'new_start')
  result.update(published=True,entry_sha256=sha(ENTRY),published_unix=time.time())
  run(['xwd','-root','-silent','-out',PUB/'screen.xwd'],PUB/'screenshot.log',8,env=ENV)
  atomic(PUB/'receipt.json',result);mark(complete=True);os.waitpid(watcher,0)
 except BaseException:
  result.update(published=False,error=traceback.format_exc())
  try:result['rollback']=rollback('parent');mark(complete=True);os.waitpid(watcher,0)
  except BaseException:result['rollback_error']=traceback.format_exc()
  atomic(PUB/'failed.json',result)
 print(json.dumps(result,indent=2),flush=True)
 if not result.get('published'):sys.exit(1)
def main():
 global OUT,STATE
 mode=sys.argv[1] if len(sys.argv)>1 else 'publish'
 with open('/userdata/codex_npu_test.lock','a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
  if mode=='publish':publish();return
  assert mode in ('stop','rollback')
  OUT=R/(mode+'_'+str(time.time_ns()));OUT.mkdir();STATE=OUT/'state.json'
  atomic(STATE,{'parent':identity(os.getpid()),'deadline':time.monotonic()+180,'complete':False,'capture_unconfirmed':False})
  if mode=='stop':stop_current('requested_stop');result={'stopped':True,'default_entry_unchanged':True}
  else:result={'restored_previous':True,'running':rollback('requested_rollback')}
  mark(complete=True);atomic(OUT/'receipt.json',result);print(json.dumps(result,indent=2))
if __name__=='__main__':main()
