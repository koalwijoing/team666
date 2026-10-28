import os, sys, time, json, signal, subprocess
from pathlib import Path

def identity(pid):
    try:
        p=Path('/proc')/str(pid); raw=(p/'stat').read_text(); s=raw[raw.rfind(')')+2:].split()
        return {'pid':pid,'start':int(s[19]),'state':s[0],'pgid':int(s[2]),'sid':int(s[3]),'exe':os.readlink(p/'exe')}
    except (OSError,ValueError): return None

def members(pgid):
    return [x for p in Path('/proc').iterdir() if p.name.isdecimal()
            for x in [identity(int(p.name))] if x and x['pgid']==pgid and x['sid']==pgid and x['state']!='Z']

def cleanup(pgid):
    for sig,limit in ((signal.SIGTERM,1.5),(signal.SIGKILL,4)):
        if not members(pgid):return
        os.killpg(pgid,sig); until=time.monotonic()+limit
        while members(pgid) and time.monotonic()<until:time.sleep(.05)
    if members(pgid):raise RuntimeError('owned group did not exit: '+str(pgid))

def atomic(path,obj):
    path=Path(path); t=path.with_suffix('.tmp');t.write_text(json.dumps(obj,indent=2));t.replace(path)

def run(args, log, timeout, state=None, env=None, cwd=None):
    log=Path(log); ready=Path(str(log)+'.ready'); ready.unlink(missing_ok=True)
    with log.open('xb') as f:
        p=subprocess.Popen([sys.executable,__file__,'child',str(ready),*map(str,args)],
            stdout=f,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,
            env=env,cwd=cwd,start_new_session=True)
        item=identity(p.pid)
        if not item:raise RuntimeError('child identity unavailable')
        if state:
            s=json.loads(Path(state).read_text());s['groups'].append(item);atomic(state,s)
        owner=identity(os.getpid())
        watcher=os.fork()
        if watcher==0:
            # Also protect compile-only jobs, which have no application window.
            # The gated child cannot exec until this independent watcher exists.
            try:
                os.setsid()
                while members(p.pid):
                    current=identity(owner['pid'])
                    if not current or current['start']!=owner['start'] or current['state']=='Z':
                        cleanup(p.pid);break
                    time.sleep(.2)
            finally:os._exit(0)
        ready.write_text('go')
        timed_out=False
        try:rc=p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out=True;cleanup(p.pid);rc=p.wait(timeout=5)
        except BaseException:
            cleanup(p.pid);p.wait(timeout=5);raise
        finally:
            cleanup(p.pid);ready.unlink(missing_ok=True)
            os.waitpid(watcher,0)
    result={'args':list(map(str,args)),'rc':rc,'timed_out':timed_out,'group':item,'remaining':members(p.pid)}
    atomic(str(log)+'.result.json',result)
    if timed_out:raise TimeoutError(str(args))
    if rc:raise RuntimeError('child rc='+str(rc)+' log='+str(log))
    return result

if __name__=='__main__' and sys.argv[1]=='child':
    ready=Path(sys.argv[2]);until=time.monotonic()+8
    while not ready.exists():
        if time.monotonic()>until:sys.exit(98)
        time.sleep(.01)
    os.execvpe(sys.argv[3],sys.argv[3:],os.environ)
