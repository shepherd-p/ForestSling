"""Task-led overview, mechanical signal diagram and multi-metric evidence figures."""
import itertools,json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
from matplotlib.collections import PolyCollection
from matplotlib.colors import LogNorm
from PIL import Image
from paper_diagrams import (INK,TEAL,ORANGE,PURPLE,LIGHT,PALETTE,plain,projection,
                            arrow,draw_body,configuration,quadrotor)

def txt(fig,x,y,s,size=8.3,**kw):
    return fig.text(x,y,s,fontsize=size,color=kw.pop('color','#202D35'),**kw)

def overview(fig,data,events,unreal,bind):
    """One spatial task, with linked camera evidence rather than six equal tiles."""
    canvas=plain(fig,[0,0,1,1])
    txt(fig,.025,.968,'Geometry and inertial sensing enable coordinated obstacle passage',9.4,weight='bold')
    folder=unreal.parent/'large_pose_course_20261026_v01/4_forked_estimated'
    protocol=json.loads((folder/'PROTOCOL.json').read_text(encoding='utf-8'))
    ax=plain(fig,[.025,.430,.95,.490])
    faces=[]
    for box in protocol['obstacles']:
        center=np.asarray(box['center']);half=np.asarray(box['half']);R=np.asarray(box.get('rotation',np.eye(3)))
        for dim in range(3):
            for sign in (-1,1):
                other=[i for i in range(3) if i!=dim];corners=[]
                for u,v in [(-1,-1),(1,-1),(1,1),(-1,1)]:
                    p=np.zeros(3);p[dim]=sign*half[dim];p[other]=[u*half[other[0]],v*half[other[1]]]
                    corners.append(center+R@p)
                q=projection(corners,az=-55,el=8);faces.append((q[:,2].mean(),q[:,:2]))
    faces.sort(key=lambda f:f[0]);ax.add_collection(PolyCollection([x[1] for x in faces],
        facecolors='#D9DFE2',edgecolors='#A1ADB5',linewidths=.3,alpha=.65,zorder=1))
    m=data['time']>=data['motion_start_sec'];p=projection(data['position'][m],az=-55,el=8)
    ax.plot(p[:,0],p[:,1],color=TEAL,lw=1.2,zorder=2)
    bounds=np.vstack([f[1] for f in faces]);lo=bounds.min(0);hi=bounds.max(0)
    label_y=lo[1]-.55
    for j,event in enumerate(events):
        k=event['source_record_index'];center=data['position'][k]-data['R'][k]@data['asset']['scan_offset']
        draw_body(ax,data['asset'],data['R'][k],center,az=-55,el=8)
        aa=projection(data['anchors'][k],az=-55,el=8);uu=projection(data['endpoints'][k],az=-55,el=8)
        for a,u in zip(aa,uu):
            ax.plot([a[0],u[0]],[a[1],u[1]],color=PALETTE[j],lw=.8,zorder=4)
            quadrotor(ax,u[:2],radius=.13)
        qp=projection(data['position'][k],az=-55,el=8)
        ax.text(qp[0],label_y,f'{chr(97+j)}',ha='center',va='center',fontsize=9,weight='bold',color=PALETTE[j],
                bbox=dict(boxstyle='circle,pad=.2',fc='white',ec=PALETTE[j],lw=.7))
        ax.plot([qp[0],qp[0]],[qp[1]-.4,label_y+.3],color=PALETTE[j],lw=.6,ls=':')
    ax.set(xlim=(lo[0]-.8,hi[0]+.8),ylim=(label_y-.5,hi[1]+.4));ax.set_aspect('equal')
    txt(fig,.035,.418,'Known openings; one continuous transport',8,color=INK)
    txt(fig,.965,.418,'Recorded states in a common spatial frame',7.5,ha='right',color=INK)
    headings=['(a) Side-on passage','(b) Low-ceiling passage','(c) Opposite-side passage']
    for j,(x,event,title) in enumerate(zip([.025,.350,.675],events,headings)):
        path=unreal/'inspection_views_clear'/f"{event['name']}.png"
        ax=fig.add_axes([x,.059,.30,.285]);ax.imshow(Image.open(path).crop((400,0,1680,1080)));ax.set_axis_off()
        txt(fig,x,.366,title,8,weight='bold',color=PALETTE[j])
        txt(fig,x,.024,f"Roll {event['attitude_rpy_deg'][0]:.1f} deg",8.1)
        bind(path,'Fig. 1 linked Unreal crossing view; common spatial crop only')
    bind(folder/'PROTOCOL.json','Fig. 1 common-frame known obstacle geometry')
    bind(folder/'trajectory.npz','Fig. 1 recorded payload and supporting aircraft configurations')

def framework(fig,folder,data,events,unreal,bind):
    """A continuous mechanical information flow with explicit given/optimized quantities."""
    ax=plain(fig,[0,0,1,1]);stages=[(.02,.285,'1  Acquire'),(.365,.625,'2  Identify'),(.705,.98,'3  Reconfigure')]
    for left,right,label in stages:
        ax.plot([left,right],[.944,.944],color=INK,lw=.8)
        txt(fig,left,.968,label,9.4,weight='bold')
    cloud=np.load(folder/'scan_world.npy');cloud-=cloud.mean(0)
    p=projection(cloud);order=np.argsort(p[:,2]);ca=plain(fig,[.025,.620,.253,.247])
    ca.scatter(p[order,0],p[order,1],c=p[order,2],cmap='Blues',s=.5,linewidths=0)
    ca.set_aspect('equal');ca.autoscale();ca.margins(.03)
    txt(fig,.025,.574,'RGB-D surface and pose',8.5)
    txt(fig,.025,.524,r'$\mathcal{S},\ \hat p(t),\ \hat R(t),\ b_i$',10,color=TEAL)
    txt(fig,.025,.470,'12-view scan; calibrated mounts',8)
    ca=plain(fig,[.360,.618,.264,.251]);asset=data['asset'];draw_body(ca,asset,center=-np.asarray(asset['scan_offset']))
    ident=json.loads((folder/'IDENTIFICATION.json').read_text(encoding='utf-8'));c=projection(ident['decoded']['com'])
    ca.scatter(*c[:2],s=30,c=ORANGE,edgecolors='white',zorder=9)
    anchors=projection(np.asarray(asset['public']['anchors'])-np.asarray(asset['scan_offset']))
    for j,a in enumerate(anchors):arrow(ca,a[:2],a[:2]+np.array([(-.15 if j<2 else .15),.50]),color=PALETTE[j],scale=8)
    ca.set(xlim=(-.85,.85),ylim=(-.5,.85),aspect='equal')
    txt(fig,.365,.574,'Tensions and integral momentum',8.5)
    txt(fig,.365,.522,r'$A\theta=b,\quad\theta=B\mu,\quad\mu\geq0$',10)
    txt(fig,.365,.470,r'$\hat\theta:\ \hat m,\ \hat c,\ \hat J_O$',10,color=ORANGE)
    for x,event in [(.703,events[0]),(.845,events[1])]:
        ca=plain(fig,[x,.615,.135,.250]);configuration(ca,data,event['source_record_index'])
    txt(fig,.705,.574,'Cable forces set support geometry',8.5)
    txt(fig,.705,.522,r'$f_i\ \mapsto\ x_{i,r}(t)$',10,color=TEAL)
    txt(fig,.705,.470,'Side-on and spread formations',8)
    arrow(ax,(.286,.740),(.347,.740),color=INK,scale=10)
    arrow(ax,(.631,.740),(.690,.740),color=INK,scale=10)
    # The mechanical chain occupies one shared baseline beneath the observations.
    ax.plot([.025,.975],[.408,.408],color=LIGHT,lw=.7)
    txt(fig,.025,.369,'GIVEN',7.4,weight='bold',color=INK)
    txt(fig,.160,.369,'Known obstacles and smooth payload pose references',8.4)
    arrow(ax,(.490,.350),(.490,.311),color=INK,scale=9)
    arrow(ax,(.638,.469),(.638,.312),color=ORANGE,scale=8)
    ax.add_patch(Rectangle((.251,.157),.495,.147,facecolor='#F7F9FA',edgecolor='#BAC8CE',lw=.8))
    txt(fig,.498,.267,'Constrained cable-force optimization',8.8,weight='bold',ha='center')
    txt(fig,.498,.217,r'$\min\ \mathcal{J}(f)\quad\mathrm{s.t.}\quad G(R_r)f=Y_d\hat\theta$',11,ha='center')
    txt(fig,.498,.178,'Aperture, loading, actuation and endpoint separation',8,ha='center')
    txt(fig,.025,.262,'GEOMETRY',7.4,weight='bold',color=TEAL)
    txt(fig,.025,.214,'Surface and attachments',8.1)
    arrow(ax,(.115,.304),(.115,.325),color=TEAL,scale=7)
    arrow(ax,(.150,.243),(.241,.243),color=TEAL,scale=8)
    txt(fig,.779,.262,'EXECUTION',7.4,weight='bold',color=TEAL)
    txt(fig,.779,.214,'Endpoint / attitude control',8.1)
    arrow(ax,(.752,.243),(.771,.243),color=TEAL,scale=8)
    txt(fig,.779,.168,'50 Hz motor commands',8.1)
    arrow(ax,(.920,.151),(.920,.088),color=TEAL,scale=8)
    arrow(ax,(.920,.088),(.115,.088),color=TEAL,scale=8,linestyle=(0,(3,2)))
    arrow(ax,(.115,.088),(.115,.146),color=TEAL,scale=8,linestyle=(0,(3,2)))
    txt(fig,.498,.035,'10 Hz RGB-D requests and synchronized tension feedback',8.2,ha='center',color=TEAL)
    bind(folder/'scan_world.npy','Fig. 2 acquired surface point cloud')
    bind(folder/'IDENTIFICATION.json','Fig. 2 fitted inertial parameters')
    bind(folder/'ASSET.json','Fig. 2 attachment geometry and nonconvex load')
    bind(folder/'trajectory.npz','Fig. 2 measured support configurations')

def identification(b):
    root=b.ROOT;read=lambda p:json.loads(p.read_text(encoding='utf-8'))
    rows=read(root/'supplementary/review_20261027/INERTIAL_AUDIT.json')
    fig=plt.figure(figsize=(7.15,4.95));gs=fig.add_gridspec(2,2,width_ratios=[1.05,1],hspace=.60,wspace=.45,
                                                       left=.10,right=.985,top=.925,bottom=.10)
    labels=['3 / box','3 / irregular','4 / box','4 / irregular','4 / fork, A','4 / fork, B']
    vals=np.array([[r[k] for k in ('mass_relative_error_pct','inertia_origin_relative_error_pct','inertia_com_relative_error_pct')] for r in rows])
    ax=fig.add_subplot(gs[0,0]);im=ax.imshow(vals,cmap='YlGnBu',norm=LogNorm(.001,10),aspect='auto')
    for i in range(len(rows)):
        for j in range(3):ax.text(j,i,f'{vals[i,j]:.3f}' if j==0 else f'{vals[i,j]:.2f}',ha='center',va='center',fontsize=7.8,color='white' if vals[i,j]>1.6 else INK)
    ax.set(yticks=range(6),yticklabels=labels,xticks=range(3),xticklabels=['Mass','$J_O$','$J_C$'])
    ax.tick_params(length=0);ax.set_title('(a) Relative inertial errors (%)',loc='left',fontsize=9)
    ax=fig.add_subplot(gs[0,1]);x=np.arange(6);err=np.array([r['com_axis_error_mm'] for r in rows])
    for j,(lab,c) in enumerate(zip(['x','y','z'],PALETTE[:3])):ax.scatter(err[:,j],x+.18*(j-1),s=22,c=c,label=lab,marker=['o','s','^'][j])
    ax.axvline(0,color='#ACB8BE',lw=.7);ax.set(yticks=x,yticklabels=labels,xlabel='Signed CoM error (mm)',ylim=(5.5,-.5))
    ax.grid(axis='x',alpha=.13)
    ax.legend(frameon=False,fontsize=7,ncol=3,loc='lower right',handletextpad=.25,columnspacing=.8)
    ax.set_title('(b) CoM error components',loc='left',fontsize=9)
    ax=fig.add_subplot(gs[1,0]);prefix=b.E['prefixes'];p=read(root/'supplementary/review_20261027/OBSERVABILITY.json')
    cells=np.full((4,6),np.nan)
    for i,(n,s) in enumerate([(3,'rectangular'),(3,'irregular'),(4,'rectangular'),(4,'irregular')]):
        for j,t in enumerate([4,6,8,10,12,16]):
            record=next(v for v in p if v['case']==f'{n}_{s}_{t}s_fit');cells[i,j]=record['condition']
    ax.imshow(cells,cmap='Blues',norm=LogNorm(2,40),aspect='auto')
    for i in range(4):
        for j in range(6):ax.text(j,i,f'{cells[i,j]:.1f}',ha='center',va='center',fontsize=7.3,color='white' if cells[i,j]>12 else INK)
    ax.set(yticks=range(4),yticklabels=labels[:4],xticks=range(6),xticklabels=[4,6,8,10,12,16],xlabel='Available observation prefix (s)');ax.tick_params(length=0)
    ax.set_title('(c) Normalized condition number',loc='left',fontsize=9)
    ax=fig.add_subplot(gs[1,1]);xx=np.arange(6)
    full=np.array([r['transport_holdout']['moment_prediction_gap_rmse_Nms'] for r in rows])
    truth=np.array([r['transport_holdout']['mass_matched_uniform_moment_prediction_gap_rmse_Nms'] for r in rows])
    for j in range(6):ax.plot([j,j],[truth[j],full[j]],color='#BAC8CE',lw=1.5)
    ax.scatter(xx,truth,s=28,facecolors='white',edgecolors=ORANGE,label='Uniform, mass matched',zorder=3)
    ax.scatter(xx,full,s=22,c=TEAL,marker='D',label='Identified',zorder=4)
    ax.set(xticks=xx,xticklabels=['3B','3I','4B','4I','4F-A','4F-B'],ylabel='Moment-impulse gap (N m s)',yscale='log',ylim=(min(full)*.65,max(truth)*30))
    ax.legend(frameon=False,fontsize=7,loc='upper left');ax.grid(axis='y',alpha=.13)
    ax.set_title('(d) Held-out moment prediction',loc='left',fontsize=9)
    b.save(fig,'fig03_identification')

def validation(b):
    root=b.ROOT;path=root/'supplementary/review_20261027/independent_replay/RESULTS.json'
    extras=json.loads(path.read_text(encoding='utf-8')) if path.exists() else []
    records=[dict(event='Base transport',metrics=b.E['numerical']['metrics'])]+extras
    names=['Base','High roll','Load transient','Closest approach'][:len(records)]
    fig=plt.figure(figsize=(7.15,4.85));gs=fig.add_gridspec(2,2,left=.13,right=.98,top=.92,bottom=.12,hspace=.65,wspace=.50)
    for col,(field,title,unit) in enumerate([
        ('maximum_payload_com_difference_m','(a) Independent replay: position','mm'),
        ('maximum_payload_angle_difference_deg','(b) Independent replay: attitude','deg')]):
        ax=fig.add_subplot(gs[0,col]);values=np.array([[m[field]*(1000 if col==0 else 1) for m in r['metrics']] for r in records])
        low=max(values.min()*.7,1e-6);high=values.max()*1.1
        ax.imshow(values,cmap='YlGnBu',norm=LogNorm(low,high),aspect='auto')
        for i in range(len(records)):
            for j in range(3):ax.text(j,i,f'{values[i,j]:.3f}',ha='center',va='center',fontsize=8,color='white' if values[i,j]>np.sqrt(low*high) else INK)
        ax.set(yticks=range(len(records)),yticklabels=names,xticks=range(3),xticklabels=['1.0','0.5','0.25'],xlabel=f'MuJoCo step (ms); error in {unit}')
        ax.tick_params(length=0);ax.set_title(title,loc='left',fontsize=9)
    ax=fig.add_subplot(gs[1,0]);rows=sorted([r for r in b.E['matrix'] if r['method']=='estimated'],key=lambda r:(r['count'],r['shape']=='irregular'))
    paths=[b.COHORT/r['case'] for r in rows]+[b.LARGE]
    labs=['3 / box','3 / irregular','4 / box','4 / irregular','4 / fork']
    for i,(p,label) in enumerate(zip(paths,labs)):
        a=json.loads((p/'VISION.json').read_text(encoding='utf-8'));x=np.sort([r['processing_sec']*1000 for r in a]);y=np.arange(1,len(x)+1)/len(x)
        ax.step(x,y,color=(PALETTE+[INK])[i],label=label,lw=1)
    ax.axvline(100,color=ORANGE,ls='--',lw=.8);ax.axhline(.95,color='#A0ADB4',ls=':',lw=.7)
    ax.set(xlabel='RGB-D processing duration (ms)',ylabel='Empirical cumulative probability',xlim=(0,140),ylim=(0,1.03))
    ax.legend(frameon=False,fontsize=6.9,ncol=2,loc='lower right');ax.set_title('(c) Image-processing distributions',loc='left',fontsize=9)
    ax=fig.add_subplot(gs[1,1]);rs=rows+[b.E['courses'][1]['summary']]
    values=np.array([r['control_p95_ms'] for r in rs]);y=np.arange(5)
    ax.hlines(y,0,20,color='#E3E9EC',lw=4);ax.scatter(values,y,c=TEAL,s=30,zorder=3)
    for i,x in enumerate(values):ax.text(x+.5,i,f'{x:.2f}',va='center',fontsize=7.4)
    ax.axvline(20,color=ORANGE,ls='--',lw=.8);ax.set(xlim=(0,21),ylim=(4.6,-.6),yticks=y,yticklabels=labs,xlabel='Control p95 (ms); 20 ms update period')
    ax.set_title('(d) Feedback computation margin',loc='left',fontsize=9)
    b.save(fig,'fig09_validation')
