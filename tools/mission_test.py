#!/usr/bin/env python3
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mission.mission_loader import MissionLoader
def main():
    a=argparse.ArgumentParser(); a.add_argument('--strategy',choices=['LEFT','RIGHT'],default='LEFT'); a.add_argument('--dry-run',action='store_true'); a.add_argument('--step',type=int); a.add_argument('--from-step',type=int,dest='from_step'); a.add_argument('--to-step',type=int,dest='to_step'); x=a.parse_args(); steps=MissionLoader().steps()
    if x.step: steps=[s for s in steps if s['id']==x.step]
    elif x.from_step is not None or x.to_step is not None:
        lo=x.from_step or 1; hi=x.to_step or 999; steps=[s for s in steps if lo<=s['id']<=hi]
    print('=================================================\n KINGPHOENIX MISSION TEST\n================================================='); print('Strategy :',x.strategy); print('Mode     :','DRY RUN' if x.dry_run else 'EXECUTION SCAFFOLD','\n')
    for s in steps: print(f"STEP {s['id']:02d}  {s['type']}  {json.dumps(s,separators=(',',':'))}")
    if not x.dry_run: print('\n[SAFE] Hardware execution is not enabled in this scaffold yet.')
if __name__=='__main__':main()
