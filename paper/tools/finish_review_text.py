"""Integrate completed revision evidence into the English research article."""
from pathlib import Path
import json,re
ROOT=Path(__file__).resolve().parents[1];M=ROOT/'manuscript';S=ROOT/'supplementary/review_20261027'
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def main():
 p=M/'main.tex';text=p.read_text(encoding='utf-8')
 abstract='''Cooperative transport through spatial restrictions requires coordinated payload attitude, cable loading and aircraft formation. Visible shape determines occupied space, whereas hidden mass distribution determines the forces required to realize a configuration. This paper develops a vision- and tension-informed framework linking these quantities for rigid payloads and known obstacles. A markerless RGB-D scan establishes surface geometry and a common pose frame. Integral momentum observations and cable tensions identify physically consistent mass, center of mass and inertia. Given smooth payload pose references, conic force optimization determines compliant support endpoints and selects feasible timing and lowering candidates under aperture, actuation and separation constraints. Six nominal closed-loop simulations yield mass errors of 0.0024--0.0188 percent and center-of-mass inertia errors of 5.46--7.87 percent. A five-arm ablation separates mass, center-of-mass and inertia information: for the four-aircraft rectangular task, peak attitude error decreases from 29.66 degrees with a mass-matched uniform model to 0.73 degrees with mass and center-of-mass identification and 0.71 degrees with full identification. Three- and four-aircraft teams transport rectangular and irregular payloads. An asymmetric forked payload completes a 28 m course with successive side-on, horizontal and opposite-side orientations, with peak position and attitude errors of 3.34 cm and 2.19 degrees. Transport-only prediction, sensor-seed repetitions, independent rigid-body replay and time-step refinement provide complementary numerical evaluations. The results connect sensed inertial moments to feasible support geometry during substantial payload reorientation.'''
 text=re.sub(r'(?s)(\\begin\{abstract\}).*?(\\end\{abstract\})',lambda m:m[1]+'\n'+abstract+'\n'+m[2],text)
 p.write_text(text,encoding='utf-8')
 p=M/'identification.tex';text=p.read_text(encoding='utf-8').replace('(b) Signed CoM errors in the scan frame.','(b) Signed CoM errors in the scan frame; circles, squares and triangles denote the $x$, $y$ and $z$ components.')
 p.write_text(text,encoding='utf-8')
 p=M/'experiments.tex';text=p.read_text(encoding='utf-8')
 anchor=r'\subsection{Sequential apertures and large-angle reorientation}'
 stats=read(S/'REPEATED_SEED_STATISTICS.json');d=stats['completed_transport_distributions'];ci=stats['wilson95_all_attempts']
 block=r'''\subsection{Controlled ablations and sensor-seed repetitions}
The revision cohort uses five inertial-information levels on each of the four base configurations: a uniform-shape model with true total mass; identified mass with uniform-shape CoM and central inertia; identified mass and CoM with mass-scaled uniform-shape central inertia; full identification; and true inertial parameters. All arms use the same acquired-geometry procedure, fixed ballast, sensor seed, preparation, reference family, candidate budget and feedback gains. The true-mass arm isolates errors in the first and second moments without a mass mismatch. The oracle is an information reference, rather than a competing planner.

\begin{table*}[t]
\centering\small
\caption{Five-arm controlled ablation. Entries give peak attitude error ($^\circ$) for completed executions. R denotes candidate planning rejection; A denotes an image-age abort before transport. Asterisks mark completed executions outside the common acceptance criteria. Each cell is one initial attempt, with all attempt records retained.}
\label{tab:ablation}
\begin{tabular}{@{}clrrrrr@{}}\toprule
UAVs & Payload & True mass, uniform & Mass only & Mass + CoM & Full identification & True inertia\\\midrule
\input{table_ablation_rows.tex}\\
\bottomrule\end{tabular}
\end{table*}

Table~\ref{tab:ablation} distinguishes candidate availability from completed-transport accuracy. Both uniform-moment arms produce a candidate only for the four-aircraft rectangular configuration; its peak attitude errors are $29.66^\circ$ and $29.44^\circ$, respectively. Mass--CoM identification, full identification and true inertia produce accepted executions whenever acquisition and transport complete. In the four-aircraft rectangular case, their attitude peaks are $0.731^\circ$, $0.712^\circ$ and $0.678^\circ$. The dominant tracking gain in this task therefore follows CoM correction. The incremental effect of identifying central inertia is smaller and depends on the configuration; the three-aircraft rectangular mass--CoM arm, for example, has a slightly smaller attitude peak than its full-identification counterpart. Complete-inertia accuracy is assessed separately by Fig.~\ref{fig:identification} and transport-only prediction.

RETRY_PARAGRAPH

SEED_PARAGRAPH

\subsection{Observation and support-model sensitivity}
Two archived preparation records, the four-aircraft rectangular case and the large-angle forked case, are each refitted under four declared measurement perturbations and twenty seeds per perturbation. The 160 fits isolate added pose noise (2 mm translation and $0.15^\circ$ rotation standard deviations), tension noise and bias (0.02 N each), fixed pose-frame bias (3 mm and $0.1^\circ$), and random observation removal (20\%). These synthetic ranges define a record-space sensitivity analysis; they do not represent hardware calibration or newly executed occluded-image flights. For the forked case, CoM-error p95 is 4.76 mm with added pose noise, 5.50 mm with tension perturbation, 2.25 mm with frame bias and 0.89 mm with observation removal. The corresponding central-inertia error p95 values are 19.52\%, 14.56\%, 11.05\% and 11.60\%. The full distributions for both records are available with the analysis protocol.

Changing $\lambda$ from $10^{-10}$ through $10^{-8}$ and $10^{-6}$ to $10^{-4}$ across all six nominal records changes CoM-error norms by at most 0.038 mm and central-inertia relative errors by at most 0.092 percentage points. A separate post-execution diagnostic restricts support locations to the evaluator's occupied body material for the two forked records. Central-inertia errors become 5.85\% and 6.72\%, compared with 5.86\% and 6.77\% for the acquired-surface hull. This comparison tests the moment envelope under the same observed motion; evaluator material geometry is used only in this diagnostic.

'''
 retries=[r for r in read(S/'TRIAL_METRICS.json') if r['retry']]
 com_retry=next((r for r in retries if r['count']==3 and r['shape']=='irregular' and r['method']=='mass_com'),None)
 retry=r'The three-aircraft irregular mass--CoM initial attempt stops on the image-age gate before transport and is retained as A, rather than classified as planning rejection.'
 if com_retry:
  retry+=f" A separately labeled acquisition retry {'completes the task' if com_retry['passed'] else 'is recorded separately'} with a ${com_retry['peak_attitude_deg']:.2f}^\\circ$ attitude peak. It does not replace the original table entry."
 block=block.replace('RETRY_PARAGRAPH',retry)
 seed=(f"Twenty additional initial attempts use independent virtual sensor seeds for the fixed four-aircraft rectangular task. "
       f"All {stats['planning_accepted']} obtain an initial plan; {stats['passed']} complete and satisfy the execution criteria, while {stats['acquisition_aborts']} stop on the image-age gate. "
       f"The all-attempt completion-and-acceptance rate is {stats['passed']}/20 ({stats['passed']/20*100:.0f}\\%; Wilson 95\\% interval {ci[0]*100:.1f}--{ci[1]*100:.1f}\\%). "
       f"Among the {stats['completed']} complete transports, peak attitude error is ${d['peak_attitude_deg']['min']:.3f}$--${d['peak_attitude_deg']['max']:.3f}^\\circ$, and its median is ${d['peak_attitude_deg']['median']:.3f}^\\circ$. "
       f"Transport-only position RMS has median {d['position_rms_cm']['median']:.2f} cm and p95 {d['position_rms_cm']['p95']:.2f} cm; attitude RMS has median ${d['attitude_rms_deg']['median']:.3f}^\\circ$ and p95 ${d['attitude_rms_deg']['p95']:.3f}^\\circ$. "
       f"Initial planning time among complete transports has median {d['planning_sec']['median']:.2f} s and p95 {d['planning_sec']['p95']:.2f} s. "
       "These distributions are conditional on completed transport, and the initial attempts remain the statistical cohort even when a separately labeled diagnostic retry is available. The repeated task fixes shape and ballast; its seed distribution concerns sensing and acquisition timing under that configuration.")
 block=block.replace('SEED_PARAGRAPH',seed)
 if anchor not in text:raise ValueError('missing section anchor')
 if r'\subsection{Controlled ablations' in text:text=re.sub(r'(?s)\\subsection\{Controlled ablations.*?(?=\\subsection\{Sequential apertures)',lambda m:block,text)
 else:text=text.replace(anchor,block+anchor)
 text=text.replace('Complete cable segments are checked with a 0.0468 m radius including termination width and observed half-sample motion.',
  'Complete cable segments are checked with a 0.0468 m radius including termination width and observed half-sample motion. These are sampled geometric margins with the stated motion inflation; they are not a general continuous-time safety certificate under arbitrary execution disturbances.')
 p.write_text(text,encoding='utf-8')
 # Synchronize the publisher-facing title and experiment scope.
 title=re.search(r'\\title\{([^}]+)\}',(M/'main.tex').read_text(encoding='utf-8'))[1]
 p=ROOT/'editorial/cover_letter.txt';text=p.read_text(encoding='utf-8');text=re.sub(r'entitled ".*?"',lambda m:'entitled "'+title+'"',text)
 text=text.replace('The numerical evaluation includes paired identified-model and uniform-shape experiments',
 'The numerical evaluation includes a five-arm inertial-information ablation, sensor-seed repetitions and paired identified-model and uniform-shape experiments')
 text=text.replace('The article is accompanied by numerical records, analysis scripts and videos',
 'The article is accompanied by the public source repository https://github.com/shepherd-p/ForestSling, numerical records, analysis scripts and videos')
 p.write_text(text,encoding='utf-8')
 print('Integrated revision evidence; abstract words:',len(re.findall(r"[A-Za-z0-9]+(?:[-'][A-Za-z0-9]+)*",abstract)))
if __name__=='__main__':main()
