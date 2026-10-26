"""Measured failure timeline and separately instrumented process delays."""
from pathlib import Path
import sys,json
import numpy as np
ROOT=Path(__file__).resolve().parents[1];LAB=ROOT if (ROOT/'runs').is_dir() else ROOT.parent
sys.path.insert(0,str(LAB/'.deps/markerless'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def age_trace(records,ts):
 last=0.;out=[];j=0
 for t in ts:
  while j<len(records) and records[j]['received_time']<=t+1e-8:
   if records[j]['valid']:last=records[j]['exposure_time']
   j+=1
  out.append(t-last)
 return np.array(out)
def main():
 out=ROOT/'supplementary/round2_20261027';run=LAB/'runs/ast_round2_20261027'
 plt.rcParams.update({'font.family':'Arial','mathtext.fontset':'stix','font.size':8,'pdf.fonttype':42,'svg.fonttype':'none','axes.spines.right':False,'axes.spines.top':False})
 fig,axs=plt.subplots(1,3,figsize=(10.3,2.8),layout='constrained')
 old=LAB/'runs/ast_review_20261027/4_rectangular_estimated_27000';new=run/'4_rectangular_estimated_27000'
 end=read(old/'SUMMARY.json')['physics_time_sec'];t=np.arange(end-3,end+.001,.02)
 for source,label,color in [(old,'Original thread','#CB7933'),(new,'Isolated process','#087E8B')]:
  axs[0].plot(t-end,age_trace(read(source/'VISION.json'),t)*1000,label=label,color=color,lw=1)
 axs[0].axhline(350,ls='--',lw=.8,c='#647681');axs[0].set(xlabel='Time relative to original abort (s)',ylabel='Latest valid exposure age (ms)',title='(a) Seed 27000: age-gate timeline');axs[0].legend(frameon=False,fontsize=7)
 delays={k:[] for k in ('queue_sec','processing_sec','return_and_read_sec')}
 for p in sorted(run.glob('4_rectangular_estimated_27*/VISION.json')):
  for v in read(p):
   for key in delays:delays[key].append(v[key]*1000)
 for (key,x),name,c in zip(delays.items(),['Queue / dispatch','Image pipeline','Return / control read'],['#7954A1','#087E8B','#CB7933']):
  x=np.sort(x);axs[1].plot(x,np.arange(1,len(x)+1)/len(x),c=c,label=name,lw=1)
 axs[1].set(xlabel='Wall-clock delay (ms)',ylabel='Empirical cumulative probability',title='(b) New process: delay components');axs[1].legend(frameon=False,fontsize=7,loc='lower right')
 oldstats=read(ROOT/'supplementary/review_20261027/REPEATED_SEED_STATISTICS.json');newstats=read(out/'REPEATED_SEED_STATISTICS.json')
 for j,(s,name,c) in enumerate([(oldstats,'Original','#CB7933'),(newstats,'Process','#087E8B')]):
  v=s['passed']/20;lo,hi=s['wilson95_all_attempts'];axs[2].errorbar(v,j,xerr=[[max(0.,v-lo)],[max(0.,hi-v)]],fmt='o',c=c,capsize=3)
  axs[2].text(.98,j+.12,f"{s['passed']}/20",ha='right',fontsize=8,color=c)
 axs[2].set(xlim=(.5,1.025),ylim=(1.5,-.5),yticks=[0,1],yticklabels=['Original','Process'],xlabel='All-attempt accepted fraction',title='(c) Same seeds: Wilson 95% intervals')
 for ext in ('pdf','svg','png'):fig.savefig(out/('S1_sensor_timing.'+ext),dpi=260,bbox_inches='tight')
 plt.close(fig)
if __name__=='__main__':main()
