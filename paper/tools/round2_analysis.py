"""Second-round auditable residuals, experiment identities and controlled plans."""
from pathlib import Path
import json,sys,csv,math
import numpy as np
ROOT=Path(__file__).resolve().parents[1];LAB=ROOT if (ROOT/'runs').is_dir() else ROOT.parent
sys.path[:0]=[str(LAB),str(LAB/'research/vision_tension_20261025')]
from forestlab.inertial_identification import parameters,decode,integral_system,wrench_matrix
from forestlab.shape_tension_identification import uniform_shape_parameters,support_basis
from forestlab.smooth_sling_forces import smooth_forces
from forestlab.tension_passage import PassagePlan,box_sdf,smooth,segment_box_hits
from forestlab.sling_trajectory import EndpointTrajectory
from forestlab.tension_transport import WrenchAllocator
from review_analysis import solve,scores
from round2_weighted_forces import smooth_forces as weighted_forces
OUT=ROOT/'supplementary/round2_20261027';OUT.mkdir(exist_ok=True)
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def write(name,d):
 def enc(x):
  if isinstance(x,np.ndarray):return x.tolist()
  if isinstance(x,np.generic):return x.item()
  raise TypeError(type(x).__name__)
 (OUT/name).write_text(json.dumps(d,indent=2,default=enc,allow_nan=False)+'\n',encoding='utf-8')
def geometry(plan,profile,anchors,cloud,obstacles):
 times=np.arange(0,plan.duration+.001,.02)
 poses=[plan.pose(t) for t in times];p=np.array([v[0] for v in poses]);R=np.array([v[1] for v in poses])
 x=profile.endpoint_curve(times);forces=profile.force_curve(times)
 load=p[:,None,:]+np.einsum('tij,nj->tni',R,cloud)
 attachments=p[:,None,:]+np.einsum('tij,nj->tni',R,anchors)
 rope_hit=any(bool(segment_box_hits(attachments,x,b,.035).any()) for b in obstacles)
 gap=min(float(box_sdf(x,b,.27).min()) for b in obstacles)
 load_gap=min(float(box_sdf(load,b,.04).min()) for b in obstacles)
 separation=min(float(np.linalg.norm(x[:,i]-x[:,j],axis=1).min()) for i in range(len(anchors)) for j in range(i))
 return dict(min_aircraft_clearance_m=gap,min_surface_clearance_m=load_gap,min_aircraft_center_distance_m=separation,
  peak_tension_N=float(np.linalg.norm(forces,axis=2).max()),min_tension_N=float(np.linalg.norm(forces,axis=2).min()),
  max_endpoint_acceleration_m_s2=profile.maximum_reference_acceleration,
  nodal_wrench_feasible=True,sampled_dynamic_accepted=bool(not rope_hit and separation>=.52 and gap>.005 and load_gap>.005 and profile.maximum_reference_acceleration<=2.25
       and profile.maximum_reference_speed<=2.5 and profile.maximum_tilt<37 and profile.maximum_thrust<28
       and profile.minimum_reference_tension>1 and profile.maximum_reference_tension<8.25)),x,forces
def main():
 nominal=LAB/'runs/vision_tension_joint_matrix_20261025_v03'
 folders=[nominal/f'{n}_{s}_estimated' for n in (3,4) for s in ('rectangular','irregular')]
 folders += [LAB/'runs/complex_forked_corner_lugs_20261026_v05/4_forked_estimated',LAB/'runs/large_pose_course_20261026_v01/4_forked_estimated']
 residuals=[];normalization=[]
 for folder in folders:
  asset=read(folder/'ASSET.json');ident=read(folder/'IDENTIFICATION.json');summary=read(folder/'SUMMARY.json')
  off=np.array(asset['scan_offset']);anchors=np.array(asset['calibrated_anchors']);h=asset['evaluator']
  truth=parameters(h['mass'],np.array(h['com'])-off,h['inertia']);full=np.array(ident['theta'])
  cloud=np.load(folder/'scan_world.npy')-off-[0,0,3]
  uniform=uniform_shape_parameters(cloud,mass=h['mass'])
  t0=summary['motion_start_sec'];rows=[r for r in read(folder/'SENSOR_ROWS.json') if t0+1<=r['time']<=t0+summary['planned_motion_duration_sec']-1]
  system=integral_system(rows[::5],anchors,.4);A=system['A'];b=system['b']
  for name,theta in [('Oracle',truth),('Full ID',full),('Mass-matched uniform',uniform)]:
   error=(A@theta-b).reshape(-1,6)
   residuals.append(dict(case=folder.relative_to(LAB/'runs').as_posix(),model=name,windows=len(error),
       force_impulse_RMS_Ns=np.sqrt(np.mean(error[:,:3]**2)),moment_impulse_RMS_Nms=np.sqrt(np.mean(error[:,3:]**2)),
       force_component_mean_Ns=error[:,:3].mean(0),moment_component_mean_Nms=error[:,3:].mean(0)))
  prep=integral_system(read(folder/'SMOOTHED_IDENTIFICATION_ROWS.json'),anchors,.4);B,_=support_basis(cloud)
  for length in (.25,.5,1.,2.):
   scale=np.tile([1,1,1,1/length,1/length,1/length],len(prep['b'])//6)
   theta,it=solve(prep['A']*scale[:,None],prep['b']*scale,B)
   normalization.append(dict(case=folder.name,source=folder.parent.name,characteristic_length_m=length,**scores(theta,truth)))
 write('MEASURED_IMPULSE_RESIDUALS.json',residuals);write('REGRESSION_UNIT_SENSITIVITY.json',normalization)
 # Same reference, same scan, same candidates and constraints: vary only the
 # model or the allocation policy. These are planning comparisons, not flights.
 comparisons=[]
 for folder in folders[:4]:
  a=read(folder/'ASSET.json');p=read(folder/'PLAN.json');ident=read(folder/'IDENTIFICATION.json')
  off=np.array(a['scan_offset']);anchors=np.array(a['calibrated_anchors']);cloud=np.load(folder/'scan_world.npy')-off-[0,0,3]
  h=a['evaluator'];full=np.array(ident['theta']);fd=decode(full);uniform=uniform_shape_parameters(cloud,mass=h['mass']);ud=decode(uniform)
  models={'Full ID':full,'Mass + CoM':parameters(fd['mass'],fd['com'],ud['inertia_com']*fd['mass']/ud['mass']),
          'Oracle':parameters(h['mass'],np.array(h['com'])-off,h['inertia']),
          'Mass-matched uniform':uniform,
          'CoM +10 mm x':parameters(fd['mass'],fd['com']+np.array([.01,0,0]),fd['inertia_com']),
          'CoM -10 mm x':parameters(fd['mass'],fd['com']-np.array([.01,0,0]),fd['inertia_com'])}
  protocol=read(folder/'PROTOCOL.json');obstacles=protocol['obstacles'];times=np.linspace(0,p['duration'],81)
  plan=PassagePlan(np.array(p['p0']),np.array(p['R0']),duration=p['duration'],peak=p['peak'],drop=p['drop'],formation_mode=p['formation_mode'])
  outputs={}
  for name,theta in list(models.items())+[('Nodewise allocation',full),('Weight 0.5',full),('Weight 2.0',full)]:
   row=dict(case=folder.name,model=name,duration=p['duration'],drop=p['drop'],scope='Fixed-reference planning audit; not additional closed-loop attempts')
   try:
    if name=='Nodewise allocation':
     allocator=WrenchAllocator(anchors,uav_mass=1,pressure=[1.25,1.15],maximum_tension=8.2,maximum_thrust=29.5);forces=[]
     for t in times:
      target=plan.at(t);allocator.vertical_span_limit=target.maximum_endpoint_z-target.position[2]-(anchors@target.rotation.T)[:,2]
      allocator.minimum_outward=.85+.55*smooth(t/2)
      f,_=allocator.allocate(target.rotation,wrench_matrix(target.rotation,target.omega_world,target.acceleration,target.alpha_world)@theta)
      forces.append(f)
     forces=np.array(forces)
    elif name.startswith('Weight '):forces,_=weighted_forces(plan,theta,anchors,times,smoothness_scale=float(name.split()[-1]))
    else:forces,_=smooth_forces(plan,theta,anchors,times)
    profile=EndpointTrajectory(plan,times,forces,anchors)
    metric,x,f=geometry(plan,profile,anchors,cloud,obstacles);row.update(metric)
    outputs[name]=(x,f)
   except (RuntimeError,ValueError) as e:row.update(nodal_wrench_feasible=False,sampled_dynamic_accepted=False,refusal=str(e))
   comparisons.append(row)
  ref=outputs['Full ID']
  for row in comparisons:
   if row['case']==folder.name and row['model'] in outputs:
    x,f=outputs[row['model']];dx=np.linalg.norm(x-ref[0],axis=2);df=np.linalg.norm(f-ref[1],axis=2)
    row.update(force_difference_RMS_N=np.sqrt(np.mean(df**2)),force_difference_max_N=df.max(),endpoint_difference_RMS_m=np.sqrt(np.mean(dx**2)),endpoint_difference_max_m=dx.max())
  np.savez_compressed(OUT/(folder.name+'_fixed_reference.npz'),time=np.arange(0,plan.duration+.001,.02),**{n.replace(' ','_').replace('+','plus')+'_endpoints':v[0] for n,v in outputs.items()},**{n.replace(' ','_').replace('+','plus')+'_forces':v[1] for n,v in outputs.items()})
 write('FIXED_REFERENCE_FORCE_GEOMETRY.json',comparisons)
 from round2_summary import identities
 identities()
 old=[]
 for p in (LAB/'runs/ast_review_20261027').glob('4_rectangular_estimated_27*/SUMMARY.json'):
  s=read(p)
  if s['reason']=='image_tracking_stale':
   v=read(p.parent/'VISION.json');old.append(dict(case=p.parent.name,abort_time_sec=s['physics_time_sec'],motion_start_sec=s['motion_start_sec'],gate_sec=.35,last_frames=v[-8:],decomposition_available=False,explanation='Original logs record processing and exposure/receipt times; queue and IPC timestamps were not instrumented, so no historical delay split is inferred.'))
 write('ORIGINAL_STALE_TIMELINES.json',old)
 print('Residual, scaling, common-reference and identity audits completed',flush=True)
if __name__=='__main__':main()
