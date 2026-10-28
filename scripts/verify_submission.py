#!/usr/bin/env python3
"""Check submitted source files, local include closure and matched firmware."""
import hashlib,json,re,subprocess,sys
from pathlib import Path,PurePosixPath
ROOT=Path(__file__).resolve().parents[1]
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
def main():
 arm=json.loads((ROOT/'docs/ARM_SOURCE_MANIFEST.json').read_text(encoding='utf-8'))
 fpga=json.loads((ROOT/'docs/FPGA_SOURCE_MANIFEST.json').read_text(encoding='utf-8'))
 for manifest in (arm,fpga):
  for item in manifest['files']:
   p=ROOT/item['path']
   if p.stat().st_size!=item['bytes'] or sha(p)!=item['sha256']:raise ValueError('source identity mismatch: '+item['path'])
 if arm['translation_unit_count']!=22:raise ValueError('expected 22 production compilation units')
 cmake=(ROOT/'app/CMakeLists.txt').read_text(encoding='utf-8')
 for unit in arm['translation_units']:
  if 'src/'+unit not in cmake:raise ValueError('source missing from build: '+unit)
 include_dirs=[ROOT/'app/src',ROOT/'app/includes',ROOT/'app',ROOT/'app/common',ROOT/'app/sdk']
 missing=[];include_count=0
 for p in (ROOT/'app').rglob('*'):
  if p.suffix not in ('.cpp','.c','.h','.hpp'):continue
  for name in re.findall(r'^\s*#\s*include\s*"([^"]+)"',p.read_text(encoding='utf-8',errors='replace'),re.M):
   include_count+=1
   if name.startswith('opencv2/'):continue  # Target OS package, also required by CMake.
   if not any((base/name).is_file() for base in [p.parent,*include_dirs]):missing.append((str(p.relative_to(ROOT)),name))
 if missing:raise ValueError('missing local headers: '+repr(missing))
 source_refs=0
 for project in (ROOT/'fpga/project').glob('*.pds'):
  for name in re.findall(r'\(_file\s+"([^"]+)"',project.read_text(errors='replace')):
   if name.startswith('../source/') or Path(name).suffix in ('.fdc','.pcf','.sdc'):
    if not (project.parent/name).is_file():raise ValueError('missing FPGA input: '+name)
    source_refs+=1
 sums=ROOT/'SHA256SUMS';checked=0
 if sums.exists():
  for line in sums.read_text(encoding='utf-8').splitlines():
   digest,name=line.split('  ',1);parts=PurePosixPath(name)
   if parts.is_absolute() or '..' in parts.parts or sha(ROOT.joinpath(*parts.parts))!=digest:raise ValueError('submission checksum mismatch: '+name)
   checked+=1
 child=subprocess.run([sys.executable,str(ROOT/'deployment/scripts/verify.py')],capture_output=True,text=True)
 if child.returncode:raise RuntimeError(child.stderr)
 print(json.dumps({'passed':True,'arm_translation_units':22,'local_header_references':include_count,'fpga_source_and_constraint_references':source_refs,'source_manifest_entries':len(arm['files'])+len(fpga['files']),'submission_files_checked':checked,'deployment_package_verified':True},indent=2))
if __name__=='__main__':main()
