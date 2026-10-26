"""Fine geometric sampling in physical command replay near course bottlenecks."""
from pathlib import Path
import sys,json
import numpy as np
ROOT=Path(__file__).resolve().parents[1];LAB=ROOT if (ROOT/'runs').is_dir() else ROOT.parent
sys.path.insert(0,str(LAB))
from forestlab.sixdof_sling import SixDofSling,rotation,mujoco
from forestlab.tension_passage import box_sdf,segment_box_hits
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def main():
 source=LAB/'runs/large_pose_course_20261026_v01/4_forked_estimated';asset=read(source/'ASSET.json');protocol=read(source/'PROTOCOL.json')
 for name in ('cells','half','anchors','nominal_theta'):asset['public'][name]=np.array(asset['public'][name])
 for name in ('com','inertia','weights','theta'):asset['evaluator'][name]=np.array(asset['evaluator'][name])
 raw=np.load(source/'trajectory.npz');states=raw['states'];commands=raw['commands'];q=states[:,1:36].reshape(-1,5,7)
 gap=np.full(len(states),np.inf)
 for box in protocol['obstacles']:gap=np.minimum(gap,box_sdf(q[:,1:,:3],box,.28).min(1))
 evidence=read(ROOT/'supplementary/evidence.json');events=evidence['courses'][1]['geometry']['events']
 indices=[int(np.argmin(gap))]+[v['source_record_index'] for v in events]
 cloud=np.load(source/'scan_world.npy')-[0,0,3]-np.array(asset['scan_offset'])
 out=[]
 for label,k in zip(['minimum_aircraft_clearance','side_on','low_ceiling','opposite_side'],indices):
  start=max(0,k-50);stop=min(len(commands),start+100)
  e=SixDofSling(asset['public'],asset['evaluator'],dt=.0005,obstacles=protocol['obstacles'])
  row=states[start];e.data.qpos[:]=row[1:36];e.data.qvel[:]=row[36:66];e.data.act[:]=row[66:82];e.data.time=row[0];mujoco.mj_forward(e.model,e.data)
  fine=np.inf;coarse=np.inf;surface=np.inf;cable_hits=0;max_speed=0;geometry=[]
  for j in range(start,stop):
   for sub in range(10):
    e.advance(commands[j],.002)
    p=e.data.qpos[:3];R=rotation(e.data.qpos[3:7]);aa=p+np.array(asset['public']['anchors'])@R.T
    endpoints=np.array([e.data.site_xpos[s] for s in e.site_ids]);pu=e.data.qpos.reshape(5,7)[1:,:3]
    scanned=p+np.einsum('ij,nj->ni',R,cloud+np.array(asset['scan_offset']))
    current=min(float(box_sdf(pu,box,.28).min()) for box in protocol['obstacles'])
    surface=min(surface,min(float(box_sdf(scanned,box,.045).min()) for box in protocol['obstacles']))
    hits=sum(int(segment_box_hits(aa,endpoints,box,.0468).any()) for box in protocol['obstacles']);cable_hits+=hits
    fine=min(fine,current)
    if sub==9:coarse=min(coarse,current)
    speed=np.linalg.norm(e.data.qvel.reshape(5,6)[1:,:3],axis=1).max();max_speed=max(max_speed,float(speed))
    geometry.append([float(e.data.time),current,surface])
  out.append(dict(interval=label,start_sec=float(states[start,0]),end_sec=float(states[stop,0]),integration_step_sec=.0005,geometry_step_sec=.002,
       minimum_aircraft_clearance_2ms_m=fine,minimum_aircraft_clearance_20ms_m=coarse,
       minimum_surface_clearance_m=surface,cable_intersection_samples=cable_hits,maximum_aircraft_center_speed_m_s=max_speed,
       local_half_sample_translation_bound_m=max_speed*.001,
       scope='Recorded motor-command replay initialized once per 2 s interval; no interior state resets. Fine sampled margin, not a disturbance-invariant safety guarantee.'))
  print(label,fine,coarse,cable_hits,flush=True)
 (ROOT/'supplementary/round2_20261027/FINE_CLEARANCE_REPLAY.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
