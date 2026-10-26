"""Publication figures from archived RGB images and numerical trajectories."""
from pathlib import Path
import hashlib,json,sys,itertools
import numpy as np
ROOT=Path(__file__).resolve().parents[1];LAB=ROOT if (ROOT/'runs').is_dir() else ROOT.parent
sys.path.insert(0,str(LAB/'.deps/markerless'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,FancyArrowPatch
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from PIL import Image

FIG=ROOT/'manuscript/figures';FIG.mkdir(parents=True,exist_ok=True)
E=json.loads((ROOT/'supplementary/evidence.json').read_text(encoding='utf-8'))
COHORT=LAB/'runs/vision_tension_joint_matrix_20261025_v03'
LARGE=LAB/'runs/large_pose_course_20261026_v01/4_forked_estimated'
UNREAL=LAB/'runs/unreal_large_pose_20261026_v01'
COLORS=['#087E8B','#CB7933','#487EB2','#7954A1'];BINDINGS=[]
plt.rcParams.update({'font.family':'Arial','mathtext.fontset':'stix','font.size':9,'axes.labelsize':9,
 'axes.titlesize':10,'legend.fontsize':7.6,'xtick.labelsize':8,'ytick.labelsize':8,
 'pdf.fonttype':42,'ps.fonttype':42,'axes.spines.top':False,'axes.spines.right':False,
 'axes.linewidth':.6,'lines.linewidth':1.25,'savefig.facecolor':'white'})

def read(p):return json.loads(p.read_text(encoding='utf-8'))
def bind(path,role):
    BINDINGS.append(dict(file=path.relative_to(LAB).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),role=role))
def save(fig,name):
    fig.savefig(FIG/(name+'.pdf'),bbox_inches='tight',pad_inches=.035,dpi=220)
    fig.savefig(FIG/(name+'.png'),bbox_inches='tight',pad_inches=.035,dpi=600 if name=='fig02_framework' else 300)
    with matplotlib.rc_context({'svg.fonttype':'none'}):
        fig.savefig(FIG/(name+'.svg'),bbox_inches='tight',pad_inches=.035)
    plt.close(fig)
def title(ax,letter,text):ax.set_title(f'({letter}) {text}',loc='left',pad=7)
def trajectory(folder):
    d=np.load(folder/'trajectory.npz');s=d['states'];n=read(folder/'SUMMARY.json')['count'];b=n+1
    q=s[:,1:1+7*b].reshape(-1,b,7);v=s[:,1+7*b:1+13*b].reshape(-1,b,6)
    rr=Rotation.from_quat(q[:,:,3:].reshape(-1,4)[:,[1,2,3,0]]).as_matrix().reshape(-1,b,3,3)
    asset=read(folder/'ASSET.json');offset=np.asarray(asset['scan_offset'])
    p=q[:,0,:3]+np.einsum('tij,j->ti',rr[:,0],offset)
    pu=q[:,1:,:3]+np.einsum('tnij,j->tni',rr[:,1:],[0.,0.,-.015])
    anchors=q[:,0,None,:3]+np.einsum('tij,nj->tni',rr[:,0],asset['public']['anchors'])
    return dict(time=s[:,0],position=p,uav_position=q[:,1:,:3],endpoints=pu,anchors=anchors,R=rr[:,0],
                tension=s[:,-n:],errors=d['errors'],reference=d['references'],asset=asset)

def figure01():
    from review_figures import overview
    d=trajectory(LARGE);d['motion_start_sec']=read(LARGE/'SUMMARY.json')['motion_start_sec']
    fig=plt.figure(figsize=(7.15,4.75))
    overview(fig,d,E['courses'][1]['geometry']['events'],UNREAL,bind)
    save(fig,'fig01_overview')


def figure02():
    from review_figures import framework
    with matplotlib.rc_context({'font.family':'Arial','mathtext.fontset':'stix',
                               'text.color':'#202D35','axes.labelcolor':'#202D35'}):
        fig=plt.figure(figsize=(7.15,2.80))
        framework(fig,LARGE,trajectory(LARGE),E['courses'][1]['geometry']['events'],UNREAL,bind)
        save(fig,'fig02_framework')


def figure03():
    fig,axs=plt.subplots(1,2,figsize=(7.15,2.7),layout='constrained')
    rows=sorted([r for r in E['matrix'] if r['method']=='estimated'],key=lambda r:(r['count'],r['shape']=='irregular'))
    ax=axs[0]
    for c,r in zip(COLORS,rows):
        points=[p for p in E['prefixes'] if p['count']==r['count'] and p['shape']==r['shape']]
        ax.plot([p['observation_prefix_sec'] for p in points],[1e3*p['com_error_m'] for p in points],'-o',ms=3,color=c,label=f"{r['count']} / {r['shape']}")
    ax.set(xlabel='Available observation prefix (s)',ylabel='CoM estimation error (mm)',xticks=[4,6,8,10,12,16]);ax.legend(ncol=1);title(ax,'a','Causal prefix identification')
    ax=axs[1];x=np.arange(4);width=.34
    uniform={r['case'].replace('_uniform',''):r for r in E['matrix'] if r['method']=='uniform'}
    ax.bar(x-width/2,[uniform[r['case'].replace('_estimated','')]['control_model_com_error_m']*1000 for r in rows],width,color='#CB7933',label='Uniform shape')
    ax.bar(x+width/2,[r['com_error_m']*1000 for r in rows],width,color='#087E8B',label='Vision and tension')
    ax.set(yscale='log',ylabel='CoM error (mm)',xticks=x,xticklabels=['3 / box','3 / irregular','4 / box','4 / irregular']);ax.tick_params(axis='x',labelrotation=18);ax.legend();title(ax,'b','Control model used for transport')
    save(fig,'fig03_identification')

def figure04():
    fig,axs=plt.subplots(2,2,figsize=(7.15,4.6),layout='constrained')
    for method,color,label in [('estimated','#087E8B','Vision and tension'),('uniform','#CB7933','Uniform shape')]:
        p=COHORT/f'4_rectangular_{method}';d=trajectory(p);s=read(p/'SUMMARY.json');t=d['errors'][:,0]-s['motion_start_sec']
        axs[0,0].plot(t,d['errors'][:,1]*100,color=color,label=label);axs[0,1].plot(t,d['errors'][:,2],color=color,label=label)
    for color,r in zip(COLORS,sorted([r for r in E['matrix'] if r['method']=='estimated'],key=lambda r:(r['count'],r['shape']=='irregular'))):
        d=trajectory(COHORT/r['case']);t=d['errors'][:,0]-r['motion_start_sec'];label=f"{r['count']} / {r['shape']}"
        axs[1,0].plot(t,d['errors'][:,1]*100,color=color,label=label);axs[1,1].plot(t,d['errors'][:,2],color=color,label=label)
    for i,ax in enumerate(axs.ravel()):
        ax.set(xlabel='Transport time (s)',ylabel='Position error (cm)' if i%2==0 else 'Attitude error (deg)');ax.grid(alpha=.16);ax.legend(fontsize=7)
        ax.axhline(10 if i%2==0 else 5,color='#a63c3c',ls='--',lw=.75)
    title(axs[0,0],'a','Four UAVs and rectangular payload');title(axs[0,1],'b','Effect of inertial information')
    title(axs[1,0],'c','Position tracking across configurations');title(axs[1,1],'d','Attitude tracking across configurations')
    save(fig,'fig05_tracking')

def figure05():
    fig,axs=plt.subplots(2,2,figsize=(7.15,4.3),layout='constrained')
    rows=sorted([r for r in E['matrix'] if r['method']=='estimated'],key=lambda r:(r['count'],r['shape']=='irregular'))
    for j,(ax,r) in enumerate(zip(axs.ravel(),rows)):
        d=trajectory(COHORT/r['case']);t=d['time']-r['motion_start_sec'];m=t>=0
        for i in range(r['count']):ax.plot(t[m],d['tension'][m,i],color=COLORS[i],label=f'Cable {i+1}',lw=.9)
        for bound in (.5,9.):ax.axhline(bound,color='#a63c3c',ls='--',lw=.75)
        ax.set(xlabel='Transport time (s)',ylabel='Tension (N)',ylim=(0,9.5));ax.legend(ncol=2);title(ax,chr(97+j),f"{r['count']} UAVs, {r['shape']}")
    save(fig,'fig06_tensions')

def figure06():
    d=trajectory(LARGE);s=read(LARGE/'SUMMARY.json');p=read(LARGE/'PROTOCOL.json');m=d['time']>=s['motion_start_sec']
    fig,axs=plt.subplots(2,1,figsize=(7.15,4.25),layout='constrained')
    for ax,coords,letter in zip(axs,[(0,1),(0,2)],['a','b']):
        for box in p['obstacles']:
            R=np.asarray(box.get('rotation',np.eye(3)));verts=np.array(list(itertools.product([-1,1],repeat=3)))*box['half'];verts=verts@R.T+box['center'];v=verts[:,coords];h=ConvexHull(v)
            ax.add_patch(Polygon(v[h.vertices],color='#a9b1b6',alpha=.5,lw=.5))
        ax.plot(d['position'][m,coords[0]],d['position'][m,coords[1]],color='#172f3f',lw=2,label='Payload')
        for i in range(4):ax.plot(d['uav_position'][m,i,coords[0]],d['uav_position'][m,i,coords[1]],color=COLORS[i],lw=.8,label=f'UAV {i+1}')
        ax.set(xlabel='x (m)',ylabel=('y (m)' if coords[1]==1 else 'z (m)'),xlim=(-2,31));ax.grid(alpha=.13)
        ax.set_ylim((-4,4) if coords[1]==1 else (0,6.5));title(ax,letter,'Top view' if coords[1]==1 else 'Side view')
    axs[0].legend(ncol=5,loc='upper center',fontsize=7)
    save(fig,'fig07_course')

def figure07():
    fig,axs=plt.subplots(2,2,figsize=(7.15,4.35),layout='constrained')
    for i,(n,shape) in enumerate(itertools.product((3,4),('rectangular','irregular'))):
        path=LAB/f'runs/unreal_vision_tension_20261026_v02/{n}_{shape}_estimated/frame_010.00.png'
        ax=axs.ravel()[i];ax.imshow(Image.open(path));ax.axis('off')
        title(ax,chr(97+i),f'{n} UAVs, {shape} payload')
        bind(path,'Fig. 4 base team/shape comparison; full original Unreal frame')
    save(fig,'fig04_configurations')


def figure08():
    d=trajectory(LARGE);s=read(LARGE/'SUMMARY.json');t=d['time']-s['motion_start_sec'];m=t>=0;t=t[m]
    actual=Rotation.from_matrix(d['R'][m]).as_euler('xyz',degrees=True)
    span=np.ptp(d['uav_position'][m],axis=1)
    pair=np.array([np.linalg.norm(d['uav_position'][m,i]-d['uav_position'][m,j],axis=1) for i in range(4) for j in range(i)]).min(axis=0)
    err=d['errors'];et=err[:,0]-s['motion_start_sec']
    fig,axs=plt.subplots(3,2,figsize=(7.15,5.9),layout='constrained')
    for i,label in enumerate(['Roll','Pitch','Yaw']):axs[0,0].plot(t,actual[:,i],color=COLORS[i],label=label)
    for i,label in enumerate(['x span','y span','z span']):axs[0,1].plot(t,span[:,i],color=COLORS[i],label=label)
    axs[1,0].plot(et,err[:,1]*100,color=COLORS[0]);axs[1,0].axhline(10,color='#a63c3c',ls='--',lw=.7)
    axs[1,1].plot(et,err[:,2],color=COLORS[1]);axs[1,1].axhline(5,color='#a63c3c',ls='--',lw=.7)
    for i in range(4):axs[2,0].plot(t,d['tension'][m,i],color=COLORS[i],label=f'Cable {i+1}',lw=.9)
    for bound in (.5,9):axs[2,0].axhline(bound,color='#a63c3c',ls='--',lw=.7)
    axs[2,1].plot(t,pair,color=COLORS[2]);axs[2,1].axhline(.52,color='#a63c3c',ls='--',lw=.7,label='0.52 m acceptance limit')
    labels=['Attitude (deg)','Formation span (m)','Position error (cm)','Attitude error (deg)','Tension (N)','Minimum UAV distance (m)']
    heads=['Payload reorientation','Formation dimensions','Translation tracking','Rotation tracking','Cable loading','Inter-aircraft separation']
    for i,(ax,lab,head) in enumerate(zip(axs.ravel(),labels,heads)):
        ax.set(xlabel='Transport time (s)',ylabel=lab,xlim=(0,127));ax.grid(alpha=.13);title(ax,chr(97+i),head)
        for event in E['courses'][1]['geometry']['events']:ax.axvline(event['transport_time_sec'],color='.55',ls=':',lw=.65)
        if i in (0,1,4,5):ax.legend(ncol=2,fontsize=6.8)
    save(fig,'fig08_course_dynamics')

def figure09():
    fig,axs=plt.subplots(1,2,figsize=(7.15,2.7),layout='constrained');metrics=E['numerical']['metrics'];x=[r['engine_step_sec']*1000 for r in metrics]
    ax=axs[0]
    for field,label,c in [('maximum_payload_com_difference_m','Payload CoM',COLORS[0]),('maximum_aircraft_com_difference_m','UAV CoM',COLORS[1])]:
        ax.loglog(x,[r[field]*1000 for r in metrics],'-o',color=c,ms=4,label=label)
    ax.set(xlabel='MuJoCo step (ms)',ylabel='Maximum position difference (mm)',xticks=[.25,.5,1.],xticklabels=['0.25','0.5','1.0'])
    ax.xaxis.set_minor_locator(plt.NullLocator());ax.xaxis.set_minor_formatter(plt.NullFormatter())
    ax.grid(alpha=.15);ax.legend();title(ax,'a','Independent continuous command replay')
    ax=axs[1];rows=sorted([r for r in E['matrix'] if r['method']=='estimated'],key=lambda r:(r['count'],r['shape']=='irregular'))+[E['courses'][1]['summary']]
    x=np.arange(5)
    ax.bar(x-.17,[r['image_processing_p95_ms'] for r in rows],.34,label='RGB-D processing p95',color=COLORS[0]);ax.bar(x+.17,[r['control_p95_ms'] for r in rows],.34,label='Control computation p95',color=COLORS[1])
    ax.set(ylabel='Wall-clock duration (ms)',xticks=x,xticklabels=['3 / box','3 / irregular','4 / box','4 / irregular','4 / fork']);ax.tick_params(axis='x',labelrotation=22);ax.legend();title(ax,'b','Asynchronous computation')
    save(fig,'fig09_validation')

def tables():
    rows=sorted([r for r in E['matrix'] if r['method']=='estimated'],key=lambda r:(r['count'],r['shape']=='irregular'))
    text=''
    for r in rows:
        text+=f"{r['count']} & {r['shape'].capitalize()} & {r['planned_motion_duration_sec']:.0f} & {r['com_error_m']*1000:.2f} & {r['max_position_error_m']*100:.2f} & {r['max_attitude_error_deg']:.2f} & {r['min_tension_n']:.2f}--{r['max_tension_n']:.2f} \\\\\n"
    (ROOT/'manuscript/table_matrix_rows.tex').write_text(text.rstrip().removesuffix('\\\\')+'\n',encoding='utf-8')
    text=''
    for r in rows:
        u=next(p for p in E['matrix'] if p['count']==r['count'] and p['shape']==r['shape'] and p['method']=='uniform')
        outcome='Planning rejection' if u['motion_start_sec'] is None else 'Attitude limit exceeded'
        text+=f"{r['count']} & {r['shape'].capitalize()} & {u['control_model_com_error_m']*1000:.2f} & {outcome} & Accepted \\\\\n"
    (ROOT/'manuscript/table_baseline_rows.tex').write_text(text.rstrip().removesuffix('\\\\')+'\n',encoding='utf-8')
    text=''
    for i,c in enumerate(E['courses']):
        r=c['summary'];g=c['geometry'];text+=f"{'Moderate rotation' if i==0 else 'Large rotation'} & {r['planned_motion_duration_sec']:.0f} & {r['max_position_error_m']*100:.2f} & {r['max_attitude_error_deg']:.2f} & {r['min_tension_n']:.2f}--{r['max_tension_n']:.2f} & {g['minimum_aircraft_separation_m']:.3f} & {r['initial_planning_wall_sec']:.2f} \\\\\n"
    (ROOT/'manuscript/table_course_rows.tex').write_text(text.rstrip().removesuffix('\\\\')+'\n',encoding='utf-8')

if __name__=='__main__':
    import review_figures
    figure03=lambda:review_figures.identification(sys.modules[__name__])
    figure09=lambda:review_figures.validation(sys.modules[__name__])
    for fn in (figure01,figure02,figure03,figure04,figure05,figure06,figure07,figure08,figure09):fn();print(fn.__name__,flush=True)
    tables()
    for p in (COHORT/'analysis/METRICS.json',COHORT/'causal_identification/RESULTS.json',COHORT/'numerical_crosscheck/SUMMARY.json',LARGE/'trajectory.npz'):bind(p,'Numerical figure source')
    (ROOT/'supplementary/FIGURE_SOURCES.json').write_text(json.dumps(BINDINGS,ensure_ascii=False,indent=2),encoding='utf-8')
