"""Insert second-round measured-residual and parameter-to-geometry evidence."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'supplementary/round2_20261027';M=ROOT/'manuscript'
def read(n):return json.loads((OUT/n).read_text(encoding='utf-8'))
def main():
 r=read('MEASURED_IMPULSE_RESIDUALS.json');names=['N3B','N3I','N4B','N4I','Fork A','Fork B'];cases=list(dict.fromkeys(x['case'] for x in r));lines=[]
 for case,name in zip(cases,names):
  a=[next(v for v in r if v['case']==case and v['model']==m) for m in ('Oracle','Full ID','Mass-matched uniform')]
  vals=[v['force_impulse_RMS_Ns'] for v in a]+[v['moment_impulse_RMS_Nms'] for v in a]
  lines.append(name+' & '+' & '.join(f'{v:.4f}' for v in vals)+r'\\')
 (M/'table_measured_residual_rows.tex').write_text('\n'.join(lines).rstrip('\\')+'\n',encoding='utf-8')
 t=read('FIXED_REFERENCE_FORCE_GEOMETRY.json');lines=[]
 for case,name in zip(list(dict.fromkeys(x['case'] for x in t)),names):
  a=next(v for v in t if v['case']==case and v['model']=='Full ID');b=next(v for v in t if v['case']==case and v['model']=='CoM +10 mm x')
  lines.append(f"{name} & {b['force_difference_RMS_N']:.3f} & {b['endpoint_difference_RMS_m']*100:.2f} & {a['min_aircraft_center_distance_m']:.3f}/{b['min_aircraft_center_distance_m']:.3f} & {a['min_aircraft_clearance_m']*100:.2f}/{b['min_aircraft_clearance_m']*100:.2f}"+r'\\')
 (M/'table_force_geometry_rows.tex').write_text('\n'.join(lines).rstrip('\\')+'\n',encoding='utf-8')
 p=M/'experiments.tex'
 if not p.exists():return
 s=p.read_text(encoding='utf-8')
 residual=r'''
\begin{table*}[t]
\centering\small
\caption{Transport-only measured cable-impulse residuals $A\theta-b$ on the same held-out windows. U denotes the true-mass uniform model; ID denotes full identification. RMS is taken over all windows and three components, separately for force and moment. The oracle uses the simulator's true moments.}
\label{tab:residual}
\begin{tabular}{@{}lrrrrrr@{}}\toprule
& \multicolumn{3}{c}{Force impulse RMS (N s)} & \multicolumn{3}{c}{Moment impulse RMS (N m s)}\\
Case & Oracle & ID & U & Oracle & ID & U\\\midrule
\input{table_measured_residual_rows.tex}\\\bottomrule
\end{tabular}
\end{table*}

The identified and oracle measured residuals are close, whereas the uniform-moment discrepancy is particularly pronounced in four-aircraft transportation. This comparison includes the observation-chain residual floor. The small nominal mass errors arise under bounded virtual measurement noise and the shared rigid-body model; they do not specify the accuracy of a physical RGB-D/tension installation.
'''
 anchor='\\subsection{Team size and payload geometry}'
 if '\\label{tab:residual}' not in s:s=s.replace(anchor,residual+'\n'+anchor)
 block=r'''
\subsection{From inertial parameters to support geometry}
To isolate the mechanical link, each group N reference is frozen at its recorded duration, lowering and initial payload pose. All allocations use that same reference and acquired scan, rather than selecting different motions for different models. A controlled 10 mm shift of the identified CoM along the scan $x$ axis retains its mass and central inertia. Table~\ref{tab:forcegeometry} reports the change in optimized cable forces and compliant endpoints. This is a planning sensitivity comparison, not another set of executed flights.

\begin{table*}[t]
\centering\small
\caption{Fixed-reference parameter-to-geometry comparison. Only the CoM changes by $+10$ mm in $x$; force and endpoint differences are relative to full identification. Paired entries give original/shifted values. Geometry uses aircraft endpoints and a 0.27 m aircraft envelope.}
\label{tab:forcegeometry}
\begin{tabular}{@{}lrrrr@{}}\toprule
Case & Force difference RMS (N) & Endpoint difference RMS (cm) & Min. endpoint distance (m) & Min. obstacle clearance (cm)\\\midrule
\input{table_force_geometry_rows.tex}\\\bottomrule
\end{tabular}
\end{table*}

The 10 mm parameter change produces 0.128--0.230 N RMS force changes and 3.86--13.29 cm RMS endpoint changes. This amplification includes active force and aperture constraints, rather than only an unconstrained wrench pseudoinverse. At the same reference, replacing central inertia by its mass-scaled uniform value while retaining identified mass and CoM changes endpoint RMS by less than 0.16 mm. These results independently locate the principal base-task benefit in CoM correction; they do not establish an additional tracking advantage from central-inertia identification.

A standard stagewise wrench allocator provides a separate force-planning comparison. At each of the same 81 time nodes it projects the outward preload and preceding force onto wrench equality, tension, vertical-span and nominal thrust constraints, without whole-interval derivative penalties or rate bounds. Both methods receive the full identified inertial model. After the same spline/stretch conversion, stagewise allocation has peak endpoint accelerations of 6.92--11.21 m/s$^2$, versus 0.80--1.99 m/s$^2$ for whole-interval allocation, and fails the common 2.25 m/s$^2$ candidate gate in all four frozen-reference cases. This comparison evaluates a specified standard allocation policy, rather than reproducing a complete published navigation system or asserting global infeasibility after retiming.

Multiplying all three derivative-penalty weights in Eq.~\eqref{eq:objective} by 0.5 or 2 retains the sampled force, body-geometry and dynamic gates in all eight fixed-reference checks. Tension peaks are 7.31--8.17 N and endpoint acceleration peaks are 0.67--1.99 m/s$^2$. The coefficients therefore affect the distribution of feasible support positions; the complete candidate-level records retain these alternatives and their geometry differences. The 0.25--2 m residual-length scaling test in Section 3 yields 0.27--3.68 mm CoM errors and 5.06--8.80\% central-inertia errors across the six nominal records.
'''
 anchor='\\subsection{Observation-record and support-model sensitivity}'
 if '\\label{tab:forcegeometry}' not in s:s=s.replace(anchor,block+'\n'+anchor)
 p.write_text(s,encoding='utf-8')
if __name__=='__main__':main()
