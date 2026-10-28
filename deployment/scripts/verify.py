#!/usr/bin/env python3
"""Verify the frozen package; optionally inspect the live board without capture."""
import argparse,hashlib,json,os,re,sys,time
from pathlib import Path,PurePosixPath

ROOT=Path(__file__).resolve().parents[1]
APP_SHA='3b1d686cb900673a92a36d9c6038da65c37c2c653e43cccb01784851319a3afd'
ENTRY_SHA='498534d85555ecc2d685945a9292f0be1b7e8fc4659cd32795da6ac80fdbc915'
APP='/userdata/codex_lane_interaction_release_20261028_r01/build/app'
ENTRY='/userdata/claude_qt_async_default_20261021/start_qt_async_default.sh'
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
def target_under(root,absolute):
 p=PurePosixPath(absolute)
 if not p.is_absolute() or '..' in p.parts:raise ValueError(absolute)
 base=Path(root).resolve();target=base.joinpath(*p.parts[1:])
 if not target.resolve().is_relative_to(base):raise ValueError('target escapes install root: '+str(target))
 return target
def check_package():
 m=json.loads((ROOT/'ARTIFACTS.json').read_text(encoding='utf-8'))
 seen=set()
 for item in m['artifacts']:
  p=PurePosixPath(item['path'])
  if p.is_absolute() or '..' in p.parts or item['path'] in seen:raise ValueError(item['path'])
  seen.add(item['path']);f=ROOT.joinpath(*p.parts)
  if f.is_symlink() or f.stat().st_size!=item['bytes'] or sha(f)!=item['sha256']:raise ValueError('artifact mismatch: '+str(f))
  if item.get('install'):target_under(ROOT/'unused_install_root',item['install'])
 if sha(ROOT/'bin/adas_app')!=APP_SHA or sha(ROOT/'runtime/start_qt_async_default.sh')!=ENTRY_SHA:raise ValueError('default identity mismatch')
 if len([i for i in m['artifacts'] if i['role']=='active_model'])!=5:raise ValueError('expected exactly five active models')
 sums=ROOT/'SHA256SUMS'
 if sums.exists():
  for line in sums.read_text(encoding='utf-8').splitlines():
   digest,name=line.split('  ',1);p=PurePosixPath(name)
   if p.is_absolute() or '..' in p.parts or sha(ROOT.joinpath(*p.parts))!=digest:raise ValueError('file checksum mismatch: '+name)
 return m
def installed(root,m):
 count=0
 for item in m['artifacts']:
  if not item.get('install'):continue
  p=target_under(root,item['install'])
  if not p.is_file() or sha(p)!=item['sha256']:raise ValueError('installed file mismatch: '+str(p))
  count+=1
 return count
def inspect_board(manifest):
 if sys.platform!='linux':raise RuntimeError('--board requires the Linux target')
 if sha(APP)!=APP_SHA or sha(ENTRY)!=ENTRY_SHA:raise ValueError('installed default differs')
 for item in manifest['artifacts']:
  if item['role']=='active_model' and sha(item['install'])!=item['sha256']:raise ValueError('installed model differs: '+item['path'])
 owners=[]
 for proc in Path('/proc').iterdir():
  if not proc.name.isdecimal():continue
  try:
   if any('pango_pci_driver' in os.readlink(f) for f in (proc/'fd').iterdir()):owners.append(proc)
  except OSError:pass
 if len(owners)!=1 or os.readlink(owners[0]/'exe')!=APP:raise ValueError('expected one capture owner using the accepted app')
 proc=owners[0];env=dict(x.decode().split('=',1) for x in (proc/'environ').read_bytes().split(b'\0') if b'=' in x)
 required={'P3_AD_REQUEST_WAIT_MS':'2000','P3_AD_RETRY_UNCERTAIN':'1','P3_SIM_LANE_EDGE_STRICT':'0','P3_ENABLE_CROSSWALK':'1','P3_ENABLE_CROSSWALK_PREDICTION':'1','P3_ENABLE_UFLD':'0','P3_ENABLE_SIGN':'0','P3_ENABLE_SPEED':'0','P3_LIGHT_INTERVAL':'2','P3_LANE_INTERVAL':'2','P3_OBJECT_INTERVAL':'1','P3_TEST_TWIN_MODEL':'/userdata/codex_rknn_accept_20261026_r01/models/twin_pad.rknn'}
 for k,v in required.items():
  if env.get(k)!=v:raise ValueError('profile mismatch: '+k)
 run=Path(Path('/userdata/claude_qt_async_default_20261021/latest_run.txt').read_text().strip())
 def frame():
  matches=re.findall(r'\[twinlite\] frame=(\d+)',(run/'app.log').read_text(errors='replace')[-40000:])
  if not matches:raise ValueError('no Twin frame progress evidence')
  return int(matches[-1])
 a=frame();time.sleep(3);b=frame()
 if b<=a:raise ValueError('frames did not advance')
 key=int(Path('/userdata/claude_keyinject_20261021/keyinject.pid').read_text())
 if b'/userdata/claude_keyinject_20261021/keyinject.py' not in (Path('/proc')/str(key)/'cmdline').read_bytes().split(b'\0'):raise ValueError('keyboard bridge identity mismatch')
 release=json.loads((ROOT/'RELEASE.json').read_text(encoding='utf-8'))
 note=Path('/sys/module/pango_pci_driver/notes/.note.gnu.build-id').read_bytes().hex()
 if note!=release['driver_identity']['runtime_note_hex']:raise ValueError('loaded driver build-id differs')
 proof=Path('/run/codex_p3_noosd_a98fedea.verified')
 paired=proof.exists() and proof.read_text().strip()==Path('/proc/sys/kernel/random/boot_id').read_text().strip()+' '+release['fpga_sbit_sha256']
 if env.get('P3_HW_OSD')=='0' and not paired:raise ValueError('HW OSD disabled without matching current-boot proof')
 return {'pid':int(proc.name),'frames':[a,b],'keyboard_pid':key,'driver_build_id_matches':True,'hw_osd':env.get('P3_HW_OSD'),'noosd_boot_proof_matches':paired,'capture_opened_by_checker':False,'hardware_written':False}
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--installed-root',type=Path);p.add_argument('--board',action='store_true');args=p.parse_args()
 m=check_package();result={'passed':True,'artifacts':len(m['artifacts']),'default_sha256':APP_SHA}
 if args.installed_root is not None:result['installed_files']=installed(args.installed_root,m)
 if args.board:result['board']=inspect_board(m)
 print(json.dumps(result,indent=2))
if __name__=='__main__':main()
