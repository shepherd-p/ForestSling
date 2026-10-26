"""Real wall-clock sensor worker; physics/control continue during RGB-D/ICP.

The rendering copy receives physical state solely to create camera pixels.
Only registered image pose is returned across the perception boundary.
"""
from pathlib import Path
import sys
import argparse
import time
import hashlib
import shutil
from concurrent.futures import ThreadPoolExecutor,ProcessPoolExecutor
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'.deps/markerless'),str(ROOT/'.deps/submission_review')]
from forestlab.sixdof_sling import SixDofSling,payload_asset,rotation,mujoco
from forestlab.sling_rgbd import SlingRGBD
from forestlab.inertial_identification import fit,decode
from forestlab.shape_tension_identification import fit_shape_supported,uniform_shape_parameters
from forestlab.tension_transport import TensionPoseServo
from forestlab.tension_passage import plan_beam_joint,segment_box_hits
from sixdof_transport import reference,log,exp
from sixdof_identification import save
from PIL import Image


def lagged_kinematic_rows(records,window=9):
    """Causal fixed-lag local polynomial derivatives, paired at exposure time.

    At invocation all records already arrived. No future physical state is
    used. The four-frame delay improves derivative noise without relabelling
    a delayed estimate as a current measurement.
    """
    rows=[];half=window//2
    for i in range(half,len(records)-half):
        chunk=records[i-half:i+half+1];center=records[i]
        t=np.array([r['time']-center['time'] for r in chunk]);R=np.asarray(center['rotation'])
        y=np.array([np.r_[r['position'],log(np.asarray(r['rotation'])@R.T)] for r in chunk])
        coef=np.polynomial.polynomial.polyfit(t,y,3)
        item=center.copy();item['position']=coef[0,:3];item['velocity']=coef[1,:3]
        item['rotation']=exp(coef[0,3:])@R;item['omega']=coef[1,3:]
        rows.append(item)
    return rows


def estimate_initial(records,anchors,cloud):
    rows=lagged_kinematic_rows(records)
    return rows,fit_shape_supported(rows,anchors,cloud,window_sec=.4,tension_floor=.01)


_IMAGE_WORKER=None

def initialize_image_process(xml,out,course,sensor_seed):
    global _IMAGE_WORKER
    _IMAGE_WORKER=ImageWorker(xml,out,course,sensor_seed)

def scan_image_process(qpos):
    return _IMAGE_WORKER.initialize(qpos)

def process_image(qpos,timestamp,submitted_wall):
    begin=time.perf_counter()
    result=_IMAGE_WORKER.process(qpos,timestamp)
    result.update(submitted_wall=submitted_wall,worker_start_wall=begin,
                  worker_end_wall=time.perf_counter(),queue_sec=begin-submitted_wall)
    return result

def close_image_process():
    _IMAGE_WORKER.sensor.close()

class ImageWorker:
    def __init__(self,xml,out,course=False,sensor_seed=2501):
        self.model=mujoco.MjModel.from_xml_string(xml);self.data=mujoco.MjData(self.model)
        self.sensor=SlingRGBD(self.model,tracking_stride=4 if course else 2,
                             tracking_correspondence_gate=.08,appearance_filter=course)
        self.sensor.rng=np.random.default_rng(sensor_seed)
        self.out=out;self.tracker=None

    def initialize(self,qpos):
        self.data.qpos[:]=qpos;mujoco.mj_forward(self.model,self.data)
        cloud,self.tracker,images=self.sensor.scan(self.data)
        if self.sensor.appearance is not None:save(self.out/'SCAN_APPEARANCE.json',self.sensor.appearance)
        np.save(self.out/'scan_world.npy',cloud)
        Image.fromarray(np.hstack(images[:3])).save(self.out/'scan_rgb.png')
        origin=self.tracker.registration.origin.copy()
        return dict(origin=origin,points=len(cloud),cloud=cloud-origin)

    def process(self,qpos,timestamp):
        begin=time.perf_counter();self.data.qpos[:]=qpos;self.data.time=timestamp
        mujoco.mj_forward(self.model,self.data)
        center=self.tracker._prediction(timestamp)[0] if self.tracker.last is not None else self.tracker.registration.origin
        result=self.sensor.track(self.data,self.tracker,center,timestamp)
        if result['valid']:
            result.update(velocity=self.tracker.velocity.copy(),omega=self.tracker.angular_velocity.copy())
        result.update(exposure_time=timestamp,processing_sec=time.perf_counter()-begin)
        return result


def run(out,count=4,shape='irregular',duration=8.,beam=False,method='estimated',physics_dt=.0005,course=False,corner_mounts=False,pose_mode='standard',sensor_seed=2501,perturbation='none'):
    if pose_mode=='large' and not (course and corner_mounts):raise ValueError('Large pose course requires installed corner mounts')
    task_wall_start=time.perf_counter()
    out.mkdir(parents=True,exist_ok=False)
    source_files=[Path(__file__),Path(__file__).with_name('sixdof_transport.py'),Path(__file__).with_name('sixdof_identification.py')]
    source_files += [ROOT/'forestlab'/name for name in ('tension_transport.py','tension_passage.py','sixdof_sling.py',
        'sling_rgbd.py','sling_trajectory.py','smooth_sling_forces.py','shape_tension_identification.py','inertial_identification.py','markerless.py','markerless_tracking.py','complex_sling_course.py','payload_rigging.py')]
    hashes={}
    for path in source_files:
        relative=path.relative_to(ROOT);destination=out/'source_snapshot'/relative
        destination.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,destination)
        hashes[str(relative)]=hashlib.sha256(path.read_bytes()).hexdigest()
    save(out/'SOURCE_SHA256.json',hashes)
    public,hidden=payload_asset(shape,count,'offset_z',2501,corner_mounts=corner_mounts)
    if course:
        from forestlab.complex_sling_course import course_geometry,plan_complex_course
        gates,obstacles=course_geometry(pose_mode);beam=True
    else:obstacles=[dict(center=[4.,0.,4.6],half=[.12,3.,.1])] if beam else []
    e=SixDofSling(public,hidden,seed=sensor_seed,obstacles=obstacles,dt=physics_dt)
    if perturbation=='motor_tau':e.motor_tau*=1.5
    tension_bias=np.array([.04,-.04,.04,-.04])[:count] if perturbation=='tension_bias' else np.zeros(count)
    save(out/'PROTOCOL.json',dict(sensor_worker='isolated process, one outstanding frame, unchanged 0.35 s age gate',perturbation=perturbation,count=count,shape=shape,seed=sensor_seed,ballast_seed=2501,beam=beam,course=course,duration=duration,method=method,corner_mounts=corner_mounts,pose_mode=pose_mode,
        obstacles=obstacles,obstacle_source='Declared known obstacle map for isolating perception/inertia/formation coupling',
        controller_hz=50.,rgbd_hz=10.,physics_dt_sec=physics_dt,
        tracking_pixel_stride=4 if course else 2,tracking_correspondence_gate_m=.08,
        tracking_appearance_filter=('RGB chromaticity learned from pre-rigging fixture scan' if course else 'not enabled'),
        commanded_aircraft_speed_limit_m_s=2.5 if beam else 1.6,
        commanded_aircraft_acceleration_limit_m_s2=4.5 if beam else 2.5,
        acceptance_gates=dict(position_error_m=.1,attitude_error_deg=5.,tension_n=[.5,9.],aircraft_tilt_deg=40.,
            motor_thrust_n=7.5,aircraft_speed_m_s=2.6,aircraft_acceleration_m_s2=6.,no_body_or_cable_contact=True),
        development=True,perception='Two calibrated external virtual RGB-D views, no markers or object IDs',
        planning='Asynchronous finite whole-segment candidate search; continuous local-map replanning not enabled in this test',
        scan_phase='Known static fixture, before cable rigging; subsequent image tracking includes cables'))
    scan_wall_start=time.perf_counter()
    with ProcessPoolExecutor(max_workers=1,initializer=initialize_image_process,
                             initargs=(e.xml,out,course,sensor_seed)) as pool,ProcessPoolExecutor(max_workers=1) as planning_pool:
        # The OpenGL context is created and used only in its owning worker.
        scan=pool.submit(scan_image_process,e.data.qpos.copy()).result()
        scan_wall_sec=time.perf_counter()-scan_wall_start
        scan_offset=scan['origin']-np.array([0.,0.,3.]);anchors=public['anchors']-scan_offset
        latest=dict(exposure_time=0.,position_ned=scan['origin'],rotation_scan_to_ned=np.eye(3),velocity=np.zeros(3),omega=np.zeros(3))
        start=e.cable_telemetry()['uav_position'].copy()
        state_records=[e.record()];motors_record=[];vision_records=[];id_rows=[]
        source_rows=[];references=[];errors=[];sensor_ages=[];control_times=[];schedule=[]
        future=None;future_cables=None;next_exposure=0.1;wall_start=time.perf_counter()
        dt=.02;ident=None;servo=None;reason=None;motion_t0=None;motion_initial=None
        identification_end=16.;plan=None;plan_future=None;planning_wall=None;planning_latency=None
        ident_future=None;smoothed_rows=None;failed_plan_diagnostics=None;control_theta=None
        identification_wall_start=None;identification_wall_sec=None
        total=identification_end+duration*(3. if course else 1.5)+(60. if course else 10.);cable_contact_steps=0;cable_body_contact_steps=0
        for k in range(round(total/dt)):
            t=float(e.data.time)
            if future is not None and future.done():
                result=future.result();result['received_time']=t
                result['control_read_wall']=time.perf_counter()
                result['return_and_read_sec']=result['control_read_wall']-result['worker_end_wall']
                result['exposure_to_read_sec']=result['control_read_wall']-result['submitted_wall']
                result['received_sensor_age_sec']=t-result['exposure_time']
                if perturbation=='observation_loss' and len(vision_records)%10==9:
                    result['valid']=False;result['declared_observation_loss']=True
                vision_records.append(result)
                if result['valid']:
                    latest=result
                    # Identification rows combine same-exposure image motion
                    # and tension/UAV telemetry, never receive-time geometry.
                    id_row=dict(time=result['exposure_time'],position=np.asarray(result['position_ned']),
                        velocity=result['velocity'],rotation=np.asarray(result['rotation_scan_to_ned']),omega=result['omega'],
                        uav_position=future_cables['uav_position'],tension=future_cables['tension'])
                    if t<=identification_end: id_rows.append(id_row)
                future=None
            if t>=next_exposure-1e-8 and future is None:
                future_cables=e.cable_telemetry();future_cables['tension']+=tension_bias
                future=pool.submit(process_image,e.data.qpos.copy(),t,time.perf_counter())
                next_exposure=t+.1
            cables=e.cable_telemetry();cables['tension']+=tension_bias;age=t-latest['exposure_time']
            p=np.asarray(latest['position_ned'])+latest['velocity']*age
            R=exp(latest['omega']*age)@np.asarray(latest['rotation_scan_to_ned'])
            row=dict(time=t,position=p,velocity=latest['velocity'],rotation=R,omega=latest['omega'],
                uav_position=cables['uav_position'],tension=cables['tension'])
            if age>.35:
                reason='image_tracking_stale'
                save(out/'STALE_EVENT.json',dict(time=t,latest_exposure=latest['exposure_time'],age_sec=age,
                    gate_sec=.35,pending=future is not None,completed_frames=len(vision_records),
                    behavior='Terminate this simulated execution at the gate; no unvalidated blind continuation or automatic retry'))
                break
            tick=time.perf_counter()
            target=None
            if t>=identification_end-1e-8 and servo is None:
                if ident_future is None:
                    identification_wall_start=time.perf_counter()
                    ident_future=planning_pool.submit(estimate_initial,id_rows.copy(),anchors.copy(),scan['cloud'].copy())
                if ident_future.done():
                    smoothed_rows,ident=ident_future.result()
                    identification_wall_sec=time.perf_counter()-identification_wall_start
                    if not ident['activated'] and method=='estimated': reason='identification_refused';break
                    from forestlab.inertial_identification import parameters
                    uniform=uniform_shape_parameters(scan['cloud']);ud=decode(uniform);ed=decode(ident['theta'])
                    if method=='unconstrained': control_theta=ident['unconstrained_theta']
                    elif method=='estimated': control_theta=ident['theta']
                    elif method=='mass_matched': control_theta=uniform*(hidden['mass']/uniform[0])
                    elif method=='mass_only': control_theta=uniform*(ed['mass']/uniform[0])
                    elif method=='mass_com': control_theta=parameters(ed['mass'],ed['com'],ud['inertia_com']*ed['mass']/ud['mass'])
                    elif method=='oracle': control_theta=parameters(hidden['mass'],hidden['com']-scan_offset,hidden['inertia'])
                    else: control_theta=uniform
                    servo=TensionPoseServo(anchors,control_theta,rest_length=e.rest_length,stiffness=e.stiffness,
                        damping=e.damping,uav_mass=e.uav_mass,pressure=1.25,
                        acceleration_limit=2.5,speed_limit=1.6)
                    delta=cables['uav_position']-p-anchors@R.T
                    servo.allocator.previous=delta/np.linalg.norm(delta,axis=1)[:,None]*cables['tension'][:,None]
                    motion_initial=(p.copy(),R.copy())
                    if beam:
                        planning_wall=time.perf_counter()
                        planner=plan_complex_course if course else plan_beam_joint
                        plan_future=planning_pool.submit(planner,p.copy(),R.copy(),control_theta.copy(),
                            anchors.copy(),scan['cloud'].copy(),obstacles,duration,servo.allocator.previous.copy(),
                            **(dict(pose_mode=pose_mode) if course else {}))
                    else:motion_t0=t
            if servo is None:
                f=e.excitation_force(start,t,amplitude=.6)
            else:
                if beam and plan is None:
                    if plan_future.done():
                        try:plan,candidates=plan_future.result()
                        except RuntimeError as error:
                            planning_latency=time.perf_counter()-planning_wall
                            reason=str(error);failed_plan_diagnostics=getattr(error,'diagnostics',None);break
                        planning_latency=time.perf_counter()-planning_wall
                        motion_t0=t
                    else:
                        f=e.excitation_force(start,t,amplitude=.6)
                if motion_t0 is not None:
                    target=plan.at(t-motion_t0+dt) if beam else reference(t-motion_t0+dt,*motion_initial,duration)
                    if beam:servo.allocator.pressure=target.pressure
                    try:
                        if beam:f,info=plan.profile.command(row,cables['uav_velocity'],t-motion_t0+dt,dt)
                        else:f,info=servo.command(row,cables['uav_velocity'],target,dt)
                    except RuntimeError as error:reason=str(error);break
                    references.append(np.r_[target.position,target.rotation.ravel()])
            command=e.motor_command(f);control_times.append(time.perf_counter()-tick)
            e.advance(command,dt);state_records.append(e.record());motors_record.append(command)
            source_rows.append(row);sensor_ages.append(age)
            if target is not None:
                Rt=rotation(e.data.qpos[3:7]);pt=e.data.qpos[:3]+Rt@scan_offset
                errors.append([e.data.time,float(np.linalg.norm(pt-target.position)),float(np.degrees(np.linalg.norm(log(Rt@target.rotation.T))))])
            Rt=rotation(e.data.qpos[3:7]);a=e.data.qpos[:3]+public['anchors']@Rt.T
            endpoints=np.array([e.data.site_xpos[sid] for sid in e.site_ids])
            for box in obstacles:
                cable_contact_steps+=int(segment_box_hits(a,endpoints,box).any())
            if corner_mounts:
                local_endpoints=(endpoints-e.data.qpos[:3])@Rt
                cable_body_contact_steps+=int(segment_box_hits(public['anchors'][:,None,:],local_endpoints[:,None,:],
                    dict(center=public['cells'][None,:,:],half=public['half']),.006).any())
            # Wall-clock simulation: image processing proceeds concurrently;
            # the main loop keeps controlling and integrating while it runs.
            deadline=wall_start+(k+1)*dt
            remaining=deadline-time.perf_counter()
            schedule.append(max(0.,-remaining))
            if remaining>0.:time.sleep(remaining)
            active_duration=plan.duration if plan is not None else duration
            if motion_t0 is not None and e.data.time>=motion_t0+active_duration+2.-1e-8:break
        if future is not None: future.result()
        pool.submit(close_image_process).result()
    save(out/'SENSOR_ROWS.json',source_rows);save(out/'IDENTIFICATION_ROWS.json',id_rows)
    if ident is not None:
        save(out/'IDENTIFICATION.json',ident);save(out/'SMOOTHED_IDENTIFICATION_ROWS.json',smoothed_rows)
        if control_theta is not None:save(out/'CONTROL_MODEL.json',dict(method=method,theta=control_theta))
    if plan is not None:
        save(out/'PLAN.json',dict(peak=plan.peak,drop=plan.drop,duration=plan.duration,p0=plan.p0,R0=plan.R0,formation_mode=plan.formation_mode,
            candidates=candidates,initial_planning_wall_sec=planning_latency,pose_mode=pose_mode))
        np.savez_compressed(out/'planned_endpoints.npz',time=plan.profile.times,
            endpoints=plan.profile.endpoint_curve(plan.profile.times),
            velocity=plan.profile.endpoint_curve(plan.profile.times,1),
            acceleration=plan.profile.endpoint_curve(plan.profile.times,2),forces=plan.profile.force_curve(plan.profile.times))
    if failed_plan_diagnostics is not None:save(out/'PLANNING_REFUSAL.json',failed_plan_diagnostics)
    save(out/'VISION.json',vision_records)
    np.savez_compressed(out/'trajectory.npz',states=state_records,commands=motors_record,references=references,errors=errors)
    save(out/'ASSET.json',dict(public=public,evaluator=hidden,scan_offset=scan_offset,calibrated_anchors=anchors))
    (out/'model.xml').write_text(e.xml,encoding='utf-8')
    err=np.asarray(errors)
    if motion_t0 is None and reason is None:reason='preparation_timeout'
    if motion_t0 is not None and reason is None and e.data.time<motion_t0+(plan.duration if plan is not None else duration)+2.-1e-8:
        reason='motion_timeout'
    summary=dict(count=count,shape=shape,method=method,completed=reason is None,reason=reason,physics_time_sec=float(e.data.time),
        wall_time_sec=time.perf_counter()-wall_start,vision_frames=len(vision_records),
        task_wall_time_including_scan_sec=time.perf_counter()-task_wall_start,
        initialization_and_scan_wall_sec=scan_wall_sec,identification_motion_sec=identification_end,
        identification_solve_and_worker_startup_wall_sec=identification_wall_sec,
        updated_vision_frames=sum(r['valid'] for r in vision_records),identification_samples=len(id_rows),
        max_position_error_m=float(err[:,1].max()) if len(err) else None,
        max_attitude_error_deg=float(err[:,2].max()) if len(err) else None,
        min_tension_n=e.minimum_tension,max_tension_n=e.maximum_tension,contacts=e.collision_steps,
        cable_contact_steps=cable_contact_steps,cable_body_contact_steps=cable_body_contact_steps,initial_planning_wall_sec=planning_latency,
        beam=beam,course=course,pose_mode=pose_mode,motion_start_sec=motion_t0,
        planned_motion_duration_sec=plan.duration if plan is not None else duration,
        maximum_aircraft_tilt_deg=e.maximum_tilt,maximum_motor_thrust_n=e.maximum_motor,
        motor_saturation_intervals=e.motor_saturation_intervals,
        maximum_aircraft_speed_m_s=e.maximum_aircraft_speed,maximum_aircraft_acceleration_m_s2=e.maximum_aircraft_acceleration,
        sensor_age_p95_ms=float(np.quantile(sensor_ages,.95)*1000),
        image_processing_p95_ms=float(np.quantile([r['processing_sec'] for r in vision_records],.95)*1000),
        control_p95_ms=float(np.quantile(control_times,.95)*1000),
        control_deadline_lateness_p95_ms=float(np.quantile(schedule,.95)*1000),
        declared_sensor_model='Two calibrated virtual external RGB-D views; no markers or object ID masks; noisy independent tension and aircraft telemetry')
    if ident is not None:
        decoded=decode(ident['theta'])
        summary.update(estimate_activated=ident['activated'],mass_relative_error=float(abs(decoded['mass']/hidden['mass']-1)),
            com_error_m=float(np.linalg.norm(decoded['com']-(hidden['com']-scan_offset))))
    summary['passed']=bool(reason is None and summary['max_position_error_m']<=.1 and summary['max_attitude_error_deg']<=5.
        and e.minimum_tension>.5 and e.maximum_tension<9. and not e.collision_steps and cable_contact_steps==0 and cable_body_contact_steps==0
        and e.maximum_tilt<=40. and e.maximum_motor<=7.5)
    summary['passed']=bool(summary['passed'] and e.maximum_aircraft_speed<=2.6 and e.maximum_aircraft_acceleration<=6.)
    gates={
        'execution_completed':reason is None,
        'position_error':bool(len(err) and summary['max_position_error_m']<=.1),
        'attitude_error':bool(len(err) and summary['max_attitude_error_deg']<=5.),
        'minimum_tension':e.minimum_tension>.5,'maximum_tension':e.maximum_tension<9.,
        'body_contact':not e.collision_steps,'cable_contact':cable_contact_steps==0,'cable_body_contact':cable_body_contact_steps==0,
        'aircraft_tilt':e.maximum_tilt<=40.,'motor_thrust':e.maximum_motor<=7.5,
        'aircraft_speed':e.maximum_aircraft_speed<=2.6,'aircraft_acceleration':e.maximum_aircraft_acceleration<=6.}
    summary['failed_gates']=[name for name,passed in gates.items() if not passed]
    save(out/'SUMMARY.json',summary);print(summary,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--beam',action='store_true')
    p.add_argument('--count',type=int,default=4);p.add_argument('--shape',default='irregular')
    p.add_argument('--duration',type=float,default=8.)
    p.add_argument('--method',choices=['estimated','uniform','mass_matched','mass_only','mass_com','oracle','unconstrained'],default='estimated')
    p.add_argument('--physics-dt',type=float,default=.0005)
    p.add_argument('--course',action='store_true')
    p.add_argument('--corner-mounts',action='store_true')
    p.add_argument('--pose-mode',choices=['standard','large'],default='standard')
    p.add_argument('--sensor-seed',type=int,default=2501)
    p.add_argument('--perturbation',choices=['none','tension_bias','motor_tau','observation_loss'],default='none')
    a=p.parse_args();run(Path(a.out).resolve(),a.count,a.shape,a.duration,a.beam,a.method,a.physics_dt,a.course,a.corner_mounts,a.pose_mode,a.sensor_seed,a.perturbation)
