"""One-round content revision against the supplied pre-submission review."""
from pathlib import Path
import json,re
ROOT=Path(__file__).resolve().parents[1];M=ROOT/'manuscript'
def edit(name,old,new):
 p=M/name;s=p.read_text(encoding='utf-8')
 if old in s:p.write_text(s.replace(old,new),encoding='utf-8')
 elif new not in s:raise ValueError(f'Missing anchor in {name}: {old[:60]}')

edit('main.tex','Vision and Tension Informed Payload Reorientation and Formation Reconfiguration for Cooperative Aerial Transport Through Obstacles',
     'Vision- and Tension-Informed Inertial Identification and Formation Reconfiguration for Cooperative Aerial Transport Through Obstacles')
edit('introduction.tex',
 'Figure~\\ref{fig:overview} shows a four-aircraft team continuously reorienting an asymmetric forked payload through narrow, low and tilted openings.',
 'Figure~\\ref{fig:overview} presents the complete task: a scanned nonconvex payload and its supporting aircraft adopt different configurations while traversing a sequence of spatial restrictions. Here, payload reorientation means a change in its rigid-body attitude, whereas formation reconfiguration means a change in the relative aircraft endpoint positions.')
old=r'Continuous reorientation of an asymmetric forked payload carried by four UAVs. (a) Side-on passage through the narrow opening at $90.0^\circ$ roll. (b) Horizontal passage beneath the low ceiling with $90.2^\circ$ yaw. (c) Opposite-side passage through the tilted opening at $-84.2^\circ$ roll. (d),(e) Nonconvex payload, corner mounts and rope termination. (f) Measured roll during the same continuous transport, with the three crossing states marked. Images are Unreal renderings of the recorded closed-loop dynamic states.'
new=r'Cooperative transport through sequential spatial restrictions. The common-frame spatial view places three recorded payload and support configurations within the known obstacle course. Linked Unreal views show (a) side-on passage through the narrow opening, (b) a spread formation beneath the low ceiling, and (c) opposite-side passage through the tilted opening. Geometry and inertial observations supply the model used for force planning; the load attitude reference family is prescribed. The three configurations belong to one continuous closed-loop simulation.'
edit('introduction.tex',old,new)
anchor='Inertial estimation supplies the complementary information.'
addition=r'''Recent AST studies further clarify the planning context. Chai et al.~\cite{chai2026} investigate time-optimal gap traversal for collaborative aerial transportation with experimental evaluation. Sun et al.~\cite{sunlocal2026} address local obstacle avoidance and trajectory tracking for multi-quadrotor transport. Chan et al.~\cite{chan2026} formulate time-optimal planning of a cable-suspended payload quadcopter using spatial reformulation. The present objective is complementary: given an aperture-conditioned rigid-payload reference family, infer inertial moments from virtual geometry, tension and motion measurements, then propagate them through a compliant force-to-endpoint model. Timing is selected from a finite set; global time optimality and autonomous discovery of the attitude sequence are outside this formulation.

Inertial estimation supplies the complementary information.'''
edit('introduction.tex',anchor,addition)
edit('introduction.tex',
 'The main contributions are as follows. First, a sensing-to-configuration formulation',
 'The contribution is an explicit connection from identified inertial moments to constrained support geometry, evaluated under a prescribed payload reference family. First, a sensing-to-configuration formulation')
old=r'Vision and tension informed transport framework. (a) A fixture scan supplies the reference surface, followed by temporal RGB-D pose registration. (b) Synchronized motion and tension observations identify physically consistent mass, CoM and inertia. (c) These moments enter constrained force planning and the compliant force-to-endpoint map; the illustrated side-on and flattened configurations are drawn from the same forked-payload course. (d) Motor-driven execution uses visual pose correction and aircraft telemetry. The solid chain connects scan, identification, initial planning and execution; the dashed return supplies visual and force measurements. Point clouds, tension traces, body geometry and execution images use the recorded forked-payload data.'
new=r'Information flow from acquired geometry to supporting formation. Surface and pose registration supplies a common geometric frame; integral momentum and tension observations supply the inertial moments. The known openings and prescribed smooth payload references enter constrained force optimization together with the reconstructed geometry and identified moments. The compliant endpoint map converts cable forces to supporting aircraft positions, followed by motor-driven feedback execution. Point clouds and support configurations use the forked-payload records. The return path denotes visual and tension feedback, rather than online attitude-sequence search.'
edit('introduction.tex',old,new)
edit('planning.tex',
 'The load reference is represented by smooth translation and attitude functions over a selected duration $D$.',
 r'''The inputs are the reconstructed surface, calibrated attachments, known obstacle boxes, fitted inertial parameters and a prescribed smooth payload reference family. The continuous optimization variables are the three components of every cable force at each time node. Duration and lowering are selected by ordered finite candidate search. Payload position and attitude follow the selected reference; aircraft endpoints are derived from optimized forces and cable stretch. Thus force allocation and support formation adapt to the measured load, while the obstacle-crossing attitude sequence is supplied by the reference family.

The load reference is represented by smooth translation and attitude functions over a selected duration $D$.''')
edit('planning.tex',
 r'with forces in newtons and $g=9.81$ m/s$^2$.',
 r'with forces in newtons, $e_z=[0,0,1]^T$, and $g_0=9.81$ m/s$^2$ denoting the positive gravity magnitude. In the thrust cone, $m_ug_0e_z=-m_u g$ with the vector convention of Section 2. The 29.5 N cone limits nominal total thrust; four 7.5 N motor limits permit 30 N total before moment allocation. Dynamic endpoint verification applies a tighter 28 N nominal limit.')
edit('planning.tex',r'\norm{f_{ij}+m_ug e_z}',r'\norm{f_{ij}+m_ug_0 e_z}')
anchor='The course uses 121 force nodes.'
edit('planning.tex',anchor,
 r'''Feasibility is evaluated at three distinct levels. The conic solve enforces nodal wrench equalities and force cones. Candidate verification evaluates damped, interpolated endpoints and complete geometric occupancy. Closed-loop execution is assessed using the measured tracking, loading and actuation limits in Section 5. The 0.72 m separation linearization reference encourages spacing during optimization; the nonlinear candidate gate is 0.52 m. Execution reports aircraft center distance separately from clearance after subtracting the collision envelope. Preparation transients are also included in execution tension extrema; they are not subject to the transport-only force reference bound.

The course uses 121 force nodes.''')
edit('identification.tex',
 'Only observations already received enter an identification solve.',
 'Only observations already received enter an identification solve. This 0.4 s derivative lag applies to preparation-phase parameter estimation. During transport, the controller uses the most recent visual pose and a velocity-based prediction to the current control time; its reported sensor age is the exposure-to-control age, without the preparation derivative lag.')
edit('identification.tex',
 'The weights in Eq.~\\eqref{eq:fit} are one representation of the fitted moments:',
 'The envelope therefore imposes moment consistency and a spatial outer bound, rather than identifying a material-only density. The weights in Eq.~\\eqref{eq:fit} are one representation of the fitted moments:')
edit('identification.tex',
 'The support spacing is 0.08 m,',
 'No empirical measurement covariance is supplied, so the recorded regression minimizes the unwhitened integral residual. Force and moment residuals are reported separately in the reproducibility data, and the regularization sensitivity is evaluated over four orders of magnitude. The support spacing is 0.08 m,')
old=r'Inertial information from the four base configurations. (a) CoM error using only observations available up to each indicated prefix, including the fixed derivative lag. (b) CoM error of the model actually used for transport: uniform mass on the scanned support or moments identified from vision and tension. Prefix analyses reuse the corresponding observation streams.'
new=r'Inertial accuracy and predictive evaluation. (a) Relative mass and inertia errors in six recorded executions; inertia errors use the Frobenius norm. (b) Signed CoM errors in the scan frame. (c) Condition number of the column-normalized integral observation matrix using only data available by each prefix; all displayed systems have rank ten. (d) RMS difference from the true-inertia moment-impulse prediction on transport observations excluded from fitting, comparing identified and mass-matched uniform models. Fork A and B denote the moderate- and large-rotation courses. Heatmap colors use logarithmic scales; cell annotations give the numerical values.'
edit('identification.tex',old,new)

rows=json.loads((ROOT/'supplementary/review_20261027/INERTIAL_AUDIT.json').read_text(encoding='utf-8'))
mass=[r['mass_relative_error_pct'] for r in rows];inertia=[r['inertia_com_relative_error_pct'] for r in rows]
text=(f"Across the four base configurations and two forked-payload executions, mass relative error is {min(mass):.4f}--{max(mass):.4f}\\%, and the CoM-inertia Frobenius relative error is {min(inertia):.2f}--{max(inertia):.2f}\\%. "
      'True and estimated mass, three-axis CoM, all six origin-inertia entries and principal moments are provided in the public numerical records. Figure~\\ref{fig:identification}(a),(b) therefore evaluates the complete ten-parameter model rather than using CoM accuracy as a proxy for inertia accuracy. ')
edit('experiments.tex','\\subsection{CoM identification and the effect of internal mass distribution}',
 '\\subsection{Complete inertial identification and held-out prediction}\n'+text)
start='The prefix analysis in Fig.~\\ref{fig:identification}(a)'
p=M/'experiments.tex';s=p.read_text(encoding='utf-8')
if start in s:
 beg=s.index(start);end=s.index('\n\n',beg)
 s=s[:beg]+r'''Figure~\ref{fig:identification}(c) evaluates observation geometry at causal prefixes from 4 to 16 s. The normalized systems have rank ten, with condition numbers of approximately 4.4--9.0. At a 6 s prefix, the original activation criteria are satisfied in all four base cases. The prefix calculations share the original observation streams and quantify information availability during preparation.

For predictive evaluation, complete 0.4 s integral windows are formed from transport-only observations, excluding the first and last second of transport. These samples do not enter the preparation fit. Figure~\ref{fig:identification}(d) compares each model's predicted moment impulse against the true-inertia prediction on the same held-out regressor. Matching the uniform model's total mass isolates its remaining first- and second-moment errors. The repository also reports measured impulse residuals for both the fitted and true-inertia models, so parameter-model discrepancy can be distinguished from observation and integration residuals.''' +s[end:]
 p.write_text(s,encoding='utf-8')

edit('experiments.tex','Maximum thrust per motor & 7.5 N\\\\',
 r'''Maximum thrust per motor & 7.5 N\\
Motor response time constant & 25 ms\\
Aircraft linear drag coefficient & 0.2 N s/m\\
Local ICP RMSE / inlier gate & 18 mm / 0.65\\
Correspondence gate & 80 mm\\''')
edit('experiments.tex','These timings separate pretransport trajectory construction from visual processing and feedback during motion.',
 'These timings separate pretransport trajectory construction from visual processing and feedback during motion. Force planning uses Clarabel with feasibility tolerance $2\\times10^{-8}$, absolute and relative gap tolerances $10^{-7}$, and at most 100 iterations. Moment fitting uses feasibility tolerance $10^{-9}$ and absolute gap tolerance $10^{-10}$. The archived solver diagnostics retain the iteration counts for individual candidates.')
edit('experiments.tex',
 'Figure~\\ref{fig:validation}(a) shows this convergence trend.',
 'Figure~\\ref{fig:validation}(a),(b) includes this interval together with three additional two-second intervals from the large-angle course: maximum absolute roll, maximum sampled tension variation, and minimum aircraft-envelope clearance. Each interval is replayed continuously without interior state resets. At a 0.5 ms MuJoCo step, the added intervals differ from the independent RK4 replay by at most 0.365 mm in payload CoM and $0.040^\\circ$ in attitude; halving the step approximately halves these differences.')
old=r'Numerical and computation measurements. (a) Maximum position differences between MuJoCo and an independently coded continuous 2 s motor-command replay, using three MuJoCo time steps. (b) Image-processing and control-computation p95 durations for the four base configurations and the large-rotation forked-payload course.'
new=r'Numerical verification and computation distributions. (a),(b) Maximum payload CoM and attitude differences from independent two-second RK4 motor-command replay at three MuJoCo steps, including the base interval and three mechanically demanding course intervals. (c) Empirical cumulative distributions from all archived image-processing durations; the dashed vertical line marks the 100 ms RGB-D request interval. (d) Control-computation p95 values relative to the 20 ms control period. The separate computation paths do not imply 50 Hz image registration or whole-segment replanning.'
edit('experiments.tex',old,new)
edit('conclusion.tex','This paper presents a vision and tension informed framework for payload reorientation and formation reconfiguration in cooperative aerial transport.',
 'This paper develops a vision- and tension-informed connection between inertial identification and supporting formation geometry for cooperative aerial transport through known obstacles. The payload pose reference family is prescribed; conic optimization selects the cable forces, compliant support endpoints and a feasible timing candidate.')
edit('conclusion.tex',
 'Supplementary Material S1 contains the task-level numerical results, archived trajectories, figure-source records and analysis scripts underlying the reported experiments. The accompanying videos show the corresponding recorded dynamic trajectories in Unreal Engine.',
 r'The simulation and identification source code, task configurations, true and fitted inertial parameters, archived trajectories, revision experiments and figure-generation scripts are available at \url{https://github.com/shepherd-p/ForestSling}. The repository documents the analysis and rerun commands and identifies each experiment by its configuration and random seed. Recorded trajectory visualizations are provided with the reproducibility materials. Proprietary Unreal environment assets are excluded; numerical analysis and MuJoCo simulation do not require those assets.')

p=M/'references.tex';s=p.read_text(encoding='utf-8')
new=r'''\bibitem{chai2026}
Y. Chai, Z. Yang, X. Liang, J. Han, Time-optimal gap-traversal trajectory planning scheme for collaborative aerial transportation systems: An experimental research, Aerospace Science and Technology 169 (2026) 111343. \doi{10.1016/j.ast.2025.111343}.
\bibitem{sunlocal2026}
J. Sun, D. Zhang, Y. Chen, L. Mu, G. Jia, Local obstacle avoidance and trajectory tracking control of multi-quadrotor cooperative transportation system, Aerospace Science and Technology 173 (2026) 111670. \doi{10.1016/j.ast.2026.111670}.
\bibitem{chan2026}
Y. Y. Chan, Y. Shu, W.-H. Chen, K. K. H. Ng, Time-optimal motion planning of cable-suspended payload quadcopters using spatial reformulation, Aerospace Science and Technology 169 (2026) 111460. \doi{10.1016/j.ast.2025.111460}.
'''
if '\\bibitem{chai2026}' not in s:p.write_text(s.replace('\\end{thebibliography}',new+'\\end{thebibliography}'),encoding='utf-8')
print('Scientific scope, full inertial evidence, planning variables and numerical verification revised')
