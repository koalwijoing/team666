#!/usr/bin/env python3
"""Install this exact release, or stage it under an explicit alternate root."""
import argparse,datetime,os,shutil,sys
from pathlib import Path
from verify import ROOT,check_package,sha,target_under,installed

def ensure_capture_idle():
 for p in Path('/proc').iterdir():
  if not p.name.isdecimal():continue
  try:
   files=[os.readlink(f) for f in (p/'fd').iterdir()]
   maps=(p/'maps').read_text(errors='replace')
  except OSError:continue
  if any('pango_pci_driver' in f for f in files) or 'pango_pci_driver' in maps:
   raise RuntimeError('capture remains active; close normally with scripts/stop.sh before installing, PID '+p.name)
def main():
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('--root',type=Path,help='alternate staging root; omitted on Linux installs under /')
 parser.add_argument('--replace',action='store_true',help='back up and replace files whose content differs')
 args=parser.parse_args()
 if args.root is None and sys.platform!='linux':raise RuntimeError('on a PC, pass an explicit --root staging directory')
 root=(args.root or Path('/')).resolve();real=(os.name=='posix' and root==Path('/'))
 if not real and root==ROOT:raise RuntimeError('staging root must differ from repository')
 m=check_package();targets=[i for i in m['artifacts'] if i.get('install')]
 if real:
  if sys.platform!='linux' or os.geteuid()!=0:raise RuntimeError('real install requires root on the Linux board')
  ensure_capture_idle()
 conflicts=[]
 for i in targets:
  dest=target_under(root,i['install'])
  if dest.exists() and (not dest.is_file() or sha(dest)!=i['sha256']):conflicts.append(i['install'])
 if conflicts and not args.replace:raise RuntimeError('different files already exist; review and rerun with --replace: '+repr(conflicts))
 stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
 backup=target_under(root,'/userdata/team666_install_backups/'+stamp)
 changed=0
 for i in targets:
  src=ROOT/i['path'];dest=target_under(root,i['install']);dest.parent.mkdir(parents=True,exist_ok=True)
  if dest.exists() and sha(dest)==i['sha256']:
   dest.chmod(int(i['mode'],8));continue
  if dest.exists():
   previous=target_under(backup,i['install']);previous.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(dest,previous)
  temporary=dest.with_name('.'+dest.name+'.team666-'+str(os.getpid()))
  with temporary.open('xb') as f:
   with src.open('rb') as g:shutil.copyfileobj(g,f)
   f.flush();os.fsync(f.fileno())
  if sha(temporary)!=i['sha256']:raise RuntimeError('copy verification failed: '+str(temporary))
  temporary.chmod(int(i['mode'],8));os.replace(temporary,dest);changed+=1
 for absolute in ['/userdata/claude_qt_async_default_20261021/runs','/userdata/claude_keyinject_20261021']:
  target_under(root,absolute).mkdir(parents=True,exist_ok=True)
 if installed(root,m)!=len(targets):raise RuntimeError('installed file count differs')
 print('INSTALL_OK files='+str(len(targets))+' changed='+str(changed)+' root='+str(root))
 if conflicts:print('Previous differing files saved under '+str(backup))
 print('No application, driver or FPGA was started or reconfigured.')
if __name__=='__main__':main()
