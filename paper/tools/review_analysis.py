"""Post-review inertial audit, out-of-fit prediction and controlled sensor sensitivity."""
from pathlib import Path
import sys,json,time,copy
import numpy as np
ROOT=Path(__file__).resolve().parents[1];LAB=ROOT.parent
sys.path[:0]=[str(LAB),str(LAB/'.deps/markerless'),str(LAB/'.deps/submission_review'),
              str(LAB/'research/vision_tension_20261025')]
from forestlab.inertial_identification import parameters,decode,tensor,integral_system
from forestlab.shape_tension_identification import support_basis,uniform_shape_parameters
from vision_closed_loop import lagged_kinematic_rows
from scipy.spatial.transform import Rotation
import clarabel
from scipy import sparse
OUT=ROOT/'supplementary/review_20261027';OUT.mkdir(exist_ok=True)

def read(p):return json.loads(p.read_text(encoding='utf-8'))
def save(p,x):
    def encode(v):
        if isinstance(v,np.ndarray):return v.tolist()
        if isinstance(v,np.generic):return v.item()
        raise TypeError(type(v).__name__)
    p.write_text(json.dumps(x,default=encode,indent=2,allow_nan=False)+'\n',encoding='utf-8')

def scores(theta,truth):
    d,g=decode(theta),decode(truth)
    eig=np.linalg.eigvalsh(d['inertia_com']);trueig=np.linalg.eigvalsh(g['inertia_com'])
    return dict(mass_relative_error_pct=abs(d['mass']/g['mass']-1)*100,
        com_axis_error_mm=(d['com']-g['com'])*1000,com_error_mm=np.linalg.norm(d['com']-g['com'])*1000,
        inertia_origin_relative_error_pct=np.linalg.norm(tensor(theta[4:])-tensor(truth[4:]))/np.linalg.norm(tensor(truth[4:]))*100,
        inertia_com_relative_error_pct=np.linalg.norm(d['inertia_com']-g['inertia_com'])/np.linalg.norm(g['inertia_com'])*100,
        principal_inertia_relative_error_pct=(eig/trueig-1)*100,
        estimated_mass=d['mass'],true_mass=g['mass'],estimated_com=d['com'],true_com=g['com'],
        estimated_inertia_origin=tensor(theta[4:]),true_inertia_origin=tensor(truth[4:]),
        estimated_inertia_com=d['inertia_com'],true_inertia_com=g['inertia_com'])

def solve(A,b,B,regularization=1e-8):
    n=B.shape[1];D=A@B
    P=sparse.csc_matrix(D.T@D+regularization*np.eye(n));q=-D.T@b
    C=sparse.csc_matrix(np.vstack([-np.eye(n),np.ones(n),-np.ones(n)]))
    rhs=np.r_[np.zeros(n),1.5,-.7]
    st=clarabel.DefaultSettings();st.verbose=False;st.tol_feas=1e-9;st.tol_gap_abs=1e-10;st.tol_gap_rel=1e-9
    s=clarabel.DefaultSolver(P,q,C,rhs,[clarabel.NonnegativeConeT(n+2)],st).solve()
    if str(s.status) not in ('Solved','AlmostSolved'):raise RuntimeError(str(s.status))
    return B@np.asarray(s.x),s.iterations

def main():
    base=LAB/'runs/vision_tension_joint_matrix_20261025_v03'
    folders=[base/f'{n}_{s}_estimated' for n in (3,4) for s in ('rectangular','irregular')]
    folders += [LAB/'runs/complex_forked_corner_lugs_20261026_v05/4_forked_estimated',
                LAB/'runs/large_pose_course_20261026_v01/4_forked_estimated']
    metrics=[];sensitivity=[];regularization=[];occupied=[]
    for folder in folders:
        a=read(folder/'ASSET.json');ident=read(folder/'IDENTIFICATION.json');summary=read(folder/'SUMMARY.json')
        h=a['evaluator'];offset=np.asarray(a['scan_offset']);anchors=np.asarray(a['calibrated_anchors'])
        truth=parameters(h['mass'],np.asarray(h['com'])-offset,h['inertia']);theta=np.asarray(ident['theta'])
        row=dict(case=folder.parent.name+'/'+folder.name,**scores(theta,truth),rank=ident['rank'],
                 normalized_condition=ident['normalized_condition'],singular_values=ident['singular_values'])
        # A separate transport observation interval; none of these samples enter the preparation fit.
        sensors=read(folder/'SENSOR_ROWS.json');start=summary['motion_start_sec']
        rows=[r for r in sensors if start+1<=r['time']<=start+summary['planned_motion_duration_sec']-1]
        system=integral_system(rows[::5],anchors,.4);A=system['A'];b=system['b']
        pred=A@theta-b;oracle=A@truth-b
        pred=pred.reshape(-1,6);oracle=oracle.reshape(-1,6)
        row['transport_holdout']=dict(interval_sec=[rows[0]['time'],rows[-1]['time']],windows=len(pred),
            fitted_force_impulse_rmse_Ns=np.sqrt(np.mean(pred[:,:3]**2)),
            oracle_force_impulse_rmse_Ns=np.sqrt(np.mean(oracle[:,:3]**2)),
            fitted_moment_impulse_rmse_Nms=np.sqrt(np.mean(pred[:,3:]**2)),
            oracle_moment_impulse_rmse_Nms=np.sqrt(np.mean(oracle[:,3:]**2)),
            moment_prediction_gap_rmse_Nms=np.sqrt(np.mean((pred[:,3:]-oracle[:,3:])**2)))
        # Match the archived markerless frame: bounding-box midpoint, not
        # the surface-sample mean (which depends on visible point density).
        cloud0=np.load(folder/'scan_world.npy');cloud0-=offset+np.array([0.,0.,3.])
        uniform=uniform_shape_parameters(cloud0,mass=h['mass'])
        diff=(A@(uniform-truth)).reshape(-1,6)
        row['transport_holdout']['mass_matched_uniform_moment_prediction_gap_rmse_Nms']=np.sqrt(np.mean(diff[:,3:]**2))
        metrics.append(row)
        cloud=cloud0
        B,points=support_basis(cloud)
        prep=read(folder/'SMOOTHED_IDENTIFICATION_ROWS.json')
        sys0=integral_system(prep,anchors,.4)
        for lam in (1e-10,1e-8,1e-6,1e-4):
            result,iterations=solve(sys0['A'],sys0['b'],B,lam)
            regularization.append(dict(case=row['case'],lambda_value=lam,iterations=iterations,**scores(result,truth)))
        if folder in folders[-2:]:
            cells=np.asarray(a['public']['cells'])-offset
            half=np.asarray(a['public']['half'])
            keep=(np.abs(points[:,None,:]-cells[None,:,:])<=half+1e-6).all(2).any(1)
            restricted,iterations=solve(sys0['A'],sys0['b'],B[:,keep])
            occupied.append(dict(case=row['case'],support_locations_hull=len(points),
                support_locations_occupied=int(keep.sum()),iterations=iterations,**scores(restricted,truth)))
        # Reuse an observation record to isolate measurement-channel sensitivity;
        # this is separate from the newly executed closed-loop seed trials.
        if folder in (folders[2],folders[-1]):
            raw=read(folder/'IDENTIFICATION_ROWS.json')
            for mode in ('pose_noise','tension_noise','calibration_bias','dropout'):
                for seed in range(20):
                    rng=np.random.default_rng(27100+seed);rr=copy.deepcopy(raw)
                    bias=rng.normal(0,.003,3);dr=Rotation.from_rotvec(rng.normal(0,np.deg2rad(.1),3)).as_matrix()
                    ten_bias=rng.normal(0,.02,len(anchors))
                    for item in rr:
                        if mode=='pose_noise':
                            item['position']=np.asarray(item['position'])+rng.normal(0,.002,3)
                            item['rotation']=Rotation.from_rotvec(rng.normal(0,np.deg2rad(.15),3)).as_matrix()@item['rotation']
                        elif mode=='tension_noise':item['tension']=np.maximum(0,np.asarray(item['tension'])+ten_bias+rng.normal(0,.02,len(anchors)))
                        elif mode=='calibration_bias':
                            item['position']=np.asarray(item['position'])+bias
                            item['rotation']=dr@item['rotation']
                    if mode=='dropout':rr=[v for v in rr if rng.random()>=.2]
                    smooth=lagged_kinematic_rows(rr);ss=integral_system(smooth,anchors,.4)
                    th,iterations=solve(ss['A'],ss['b'],B)
                    sensitivity.append(dict(case=row['case'],mode=mode,seed=27100+seed,**scores(th,truth)))
                save(OUT/'SENSOR_SENSITIVITY.json',sensitivity)
                print('Sensitivity',folder.parent.name,folder.name,mode,flush=True)
        save(OUT/'INERTIAL_AUDIT.json',metrics);save(OUT/'REGULARIZATION.json',regularization)
        print(row['case'],{k:row[k] for k in ['mass_relative_error_pct','com_error_mm','inertia_com_relative_error_pct']},flush=True)
    prefixes=[]
    for path in sorted((base/'causal_identification').glob('*s_fit.json')):
        a=read(path);prefixes.append(dict(case=path.stem,rank=a['rank'],condition=a['normalized_condition'],singular_values=a['singular_values']))
    save(OUT/'OBSERVABILITY.json',prefixes)
    save(OUT/'OCCUPIED_SUPPORT_DIAGNOSTIC.json',dict(
        scope='Evaluator body material geometry supplied only for this post-execution support-restriction diagnostic; the online estimator retains its acquired-surface hull',
        frame='Archived scan bounding-box midpoint, matching calibrated attachments and inertial rows',results=occupied))
    save(OUT/'PROTOCOL.json',dict(uncertainty_scope='Observation-space sensitivity on archived image-derived pose records, not repeated flights or pixel-level occlusion tests',
        seeds=list(range(27100,27120)),pose_noise_m=.002,rotation_noise_deg=.15,
        tension_std_N=.02,tension_bias_std_N=.02,extrinsic_translation_std_m=.003,
        extrinsic_rotation_std_deg=.1,dropout_probability=.2,
        rationale='Declared synthetic sensitivity ranges, not hardware-calibrated specifications',
        holdout='Complete transport-only integral windows excluded from the preparation fit',
        regularization=[1e-10,1e-8,1e-6,1e-4]))

if __name__=='__main__':main()
