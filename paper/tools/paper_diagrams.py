"""Data-driven, editable vector overview and method illustrations."""
import itertools
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch, Circle, Ellipse, Polygon
from matplotlib.collections import PolyCollection
from PIL import Image

INK = '#183849'
TEAL = '#087E8B'
ORANGE = '#CB7933'
PURPLE = '#7954A1'
GRAY = '#647681'
LIGHT = '#D9E2E6'
PALETTE = [TEAL, ORANGE, '#487EB2', PURPLE]


def arrow(ax, a, b, color=INK, lw=1.1, scale=9, **kwargs):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle='-|>', mutation_scale=scale,
                               linewidth=lw, color=color, shrinkA=0, shrinkB=0, **kwargs))


def plain(fig, rect, limits=(0, 1, 0, 1)):
    ax = fig.add_axes(rect)
    ax.set(xlim=limits[:2], ylim=limits[2:])
    ax.set_axis_off()
    return ax


def projection(points, az=-48, el=28):
    a, e = np.deg2rad([az, el])
    basis = np.array([[-np.sin(a), np.cos(a), 0],
                      [-np.sin(e)*np.cos(a), -np.sin(e)*np.sin(a), np.cos(e)],
                      [np.cos(e)*np.cos(a), np.cos(e)*np.sin(a), np.sin(e)]])
    return np.asarray(points) @ basis.T


def body_faces(asset, R=None, center=None):
    public = asset['public']
    cells = np.asarray(public['cells'])
    half = np.asarray(public['half'])
    R = np.eye(3) if R is None else R
    center = np.zeros(3) if center is None else center
    # Suppress internal faces; all remaining facets describe the actual nonconvex body.
    directions = np.vstack([np.eye(3), -np.eye(3)])
    keys = {tuple(np.round(c, 6)) for c in cells}
    faces, normals = [], []
    for c in cells:
        for normal in directions:
            neighbor = c + 2*half*normal
            if tuple(np.round(neighbor, 6)) in keys:
                continue
            d = np.flatnonzero(normal)[0]
            others = [k for k in range(3) if k != d]
            verts = []
            for v, w in [(-1,-1), (1,-1), (1,1), (-1,1)]:
                p = c.copy();p[d] += half[d]*normal[d]
                p[others[0]] += half[others[0]]*v
                p[others[1]] += half[others[1]]*w
                verts.append(center + R @ p)
            faces.append(verts);normals.append(R @ normal)
    return np.asarray(faces), np.asarray(normals)


def draw_body(ax, asset, R=None, center=None, az=-48, el=28, alpha=1):
    faces, normals = body_faces(asset, R, center)
    proj = projection(faces, az, el)
    order = np.argsort(proj[:,:,2].mean(1))
    light = np.array([-.5,-.35,1]);light /= np.linalg.norm(light)
    shade = .48 + .47*np.maximum(0, normals @ light)
    rgb = np.array([.36,.57,.64])
    colors = np.column_stack([np.minimum(1, shade[:,None]*rgb[None,:]+.13),np.full(len(shade),alpha)])
    ax.add_collection(PolyCollection(proj[order,:,:2], facecolors=colors[order],
                                     edgecolors='none', linewidths=0, zorder=2))
    return proj.reshape(-1,3)


def quadrotor(ax, center, radius=.10, color=INK, zorder=6):
    x,y = center
    for dx,dy in [(-1,-1),(-1,1),(1,-1),(1,1)]:
        ax.plot([x,x+dx*radius*.67],[y,y+dy*radius*.48],color=color,lw=.85,zorder=zorder)
        ax.add_patch(Ellipse((x+dx*radius*.67,y+dy*radius*.48),radius*.82,radius*.36,
                             facecolor='white',edgecolor=color,lw=.7,zorder=zorder+1))
    ax.add_patch(Ellipse((x,y),radius*.5,radius*.32,facecolor=color,edgecolor=color,zorder=zorder+2))


def configuration(ax, d, k, az=25, el=12):
    origin = d['position'][k]
    center = -d['R'][k] @ np.asarray(d['asset']['scan_offset'])
    verts = draw_body(ax,d['asset'],d['R'][k],center,az,el)
    anchors = projection(d['anchors'][k]-origin,az,el)
    endpoints = projection(d['endpoints'][k]-origin,az,el)
    for i,(a,x) in enumerate(zip(anchors,endpoints)):
        ax.plot([a[0],x[0]],[a[1],x[1]],color=PALETTE[i],lw=1.1,zorder=3)
        arrow(ax,tuple(a[:2]+.22*(x-a)[:2]),tuple(a[:2]+.53*(x-a)[:2]),
              color=PALETTE[i],lw=1,scale=7,zorder=4)
        ax.scatter(*a[:2],s=7,c=PALETTE[i],zorder=5)
        quadrotor(ax,x[:2],.095)
    allp = np.vstack([verts[:,:2],endpoints[:,:2]])
    lo,hi = allp.min(0),allp.max(0)
    ax.set(xlim=(lo[0]-.2,hi[0]+.2),ylim=(lo[1]-.16,hi[1]+.2),aspect='equal')
    ax.set_axis_off()


def overview(fig, data, events, unreal, bind):
    """Latest course first: three physical crossing states plus construction details."""
    w=.318; xs=[.01,.342,.674]
    labels=['Side-on: narrow opening','Horizontal: low ceiling','Opposite-side: tilted opening']
    for i,(x,event,heading) in enumerate(zip(xs,events,labels)):
        path=unreal/'inspection_views_clear'/f"{event['name']}.png"
        ax=fig.add_axes([x,.47,w,.44])
        # A common spatial crop enlarges the complete team without changing pixels.
        ax.imshow(Image.open(path).crop((400,0,1680,1080)));ax.set_axis_off()
        fig.text(x,.956,f'({chr(97+i)}) {heading}',fontsize=7.6,weight='bold',color=INK)
        r,p,y=event['attitude_rpy_deg']
        fig.text(x,.438,rf'Roll ${r:.1f}^\circ$  |  Yaw ${y:.1f}^\circ$',fontsize=8,color=INK)
        bind(path,'Fig. 1 latest forked-payload crossing; spatial crop (400,0,1680,1080) only')
    for x,name,heading,letter in [
            (xs[0],'payload_closeup.png','Forked payload and mounts','d'),
            (xs[1],'rigging_attachment.png','Padeye and rope termination','e')]:
        path=unreal/'4_forked_estimated'/name
        ax=fig.add_axes([x,.075,w,.28]);ax.imshow(Image.open(path));ax.set_axis_off()
        fig.text(x,.379,f'({letter}) {heading}',fontsize=7.3,weight='bold',color=INK)
        bind(path,'Fig. 1 latest course rigging detail; full original frame')
    ax=fig.add_axes([xs[2]+.043,.095,w-.054,.238])
    from scipy.spatial.transform import Rotation
    start=data['motion_start_sec'];m=data['time']>=start
    t=data['time'][m]-start
    roll=Rotation.from_matrix(data['R'][m]).as_euler('xyz',degrees=True)[:,0]
    ax.plot(t,roll,color=TEAL,lw=1.5)
    for i,event in enumerate(events):
        et=event['transport_time_sec'];k=np.argmin(abs(t-et))
        ax.scatter(t[k],roll[k],s=21,facecolors='white',edgecolors=ORANGE,linewidth=1.1,zorder=5)
        ax.text(t[k]+3,roll[k]+(-15 if i==0 else 12),f'({chr(97+i)})',fontsize=6.5,color=INK)
    ax.set(xlabel='Transport time (s)',ylabel='Roll (deg)',xlim=(0,125),ylim=(-108,110),yticks=[-90,0,90],xticks=[0,60,120])
    ax.tick_params(labelsize=6.7,pad=2);ax.xaxis.label.set_size(7);ax.yaxis.label.set_size(7)
    ax.grid(axis='y',color=LIGHT,lw=.5);ax.spines[['top','right']].set_visible(False)
    fig.text(xs[2],.379,'(f) Measured payload roll',fontsize=7.3,weight='bold',color=INK)


def framework(fig, folder, data, events, unreal, bind):
    """Four visual panels with a mechanically meaningful causal signal flow."""
    # Panel geometry in figure coordinates. The gap is reserved for connecting arrows.
    panels={'a':(.02,.547,.446,.426),'b':(.534,.547,.446,.426),
            'd':(.02,.035,.446,.426),'c':(.534,.035,.446,.426)}
    canvas=plain(fig,[0,0,1,1])
    text_color = '#202D35'
    titles={'a':'Geometry and pose estimation','b':'Inertial identification',
            'c':'Payload and formation planning','d':'Closed-loop transport'}
    for key,(x,y,w,h) in panels.items():
        canvas.add_patch(Rectangle((x,y),w,h,facecolor='white',edgecolor=LIGHT,linewidth=.8))
        canvas.plot([x,x+w],[y+h,y+h],color=TEAL if key in 'ad' else ORANGE,lw=1.8)
        fig.text(x+.012,y+h-.037,f'({key}) {titles[key]}',fontsize=9.2,weight='bold',color=text_color)
    # (a) A cropped original camera view and the matching measured point cloud.
    path=unreal/'4_forked_estimated/payload_closeup.png'
    ax=fig.add_axes([.036,.713,.183,.175]);im=Image.open(path)
    ax.imshow(im.crop((260,130,1640,1020)));ax.set_axis_off()
    fig.text(.1275,.687,'RGB view',fontsize=8.2,color=text_color,ha='center')
    cloud=np.load(folder/'scan_world.npy');cloud-=cloud.mean(0)
    projected=projection(cloud,az=-48,el=28);order=np.argsort(projected[:,2])
    ax=plain(fig,[.244,.713,.204,.175])
    ax.scatter(projected[order,0],projected[order,1],c=projected[order,2],cmap='Blues',
               s=.65,alpha=.9,linewidths=0,rasterized=False)
    ax.set_aspect('equal');ax.autoscale();ax.margins(.03)
    fig.text(.346,.687,'Surface point cloud',fontsize=8.2,color=text_color,ha='center')
    arrow(canvas,(.221,.799),(.247,.799),color=TEAL,scale=8)
    fig.text(.039,.642,'12-view fusion and pose registration',fontsize=8.3,color=text_color)
    fig.text(.039,.592,r'$\mathcal{S},\quad \hat p(t),\ \hat R(t)$',fontsize=10,color=TEAL)
    fig.text(.255,.595,'Attachment points',fontsize=8.1,color=text_color)
    # (b) The same nonconvex body, measured force directions and estimated CoM.
    asset=data['asset'];ident=__import__('json').loads((folder/'IDENTIFICATION.json').read_text())
    ax=plain(fig,[.546,.714,.204,.173])
    mesh=draw_body(ax,asset,center=-np.asarray(asset['scan_offset']),az=-48,el=28)
    anchors=np.asarray(asset['public']['anchors'])-np.asarray(asset['scan_offset'])
    uv=projection(anchors,az=-48,el=28)
    prep_index=np.argmin(abs(data['time']-12))
    directions=(data['endpoints'][prep_index]-data['anchors'][prep_index]) @ data['R'][prep_index]
    directions/=np.linalg.norm(directions,axis=1)[:,None]
    projected_directions=projection(directions,az=-48,el=28)
    for i,a in enumerate(uv):
        arrow(ax,a[:2],a[:2]+.55*projected_directions[i,:2],PALETTE[i],scale=8,lw=1.05,zorder=4)
    c=np.asarray(ident['decoded']['com']);q=projection(c,az=-48,el=28)
    ax.scatter(*q[:2],s=25,c=ORANGE,edgecolor='white',linewidth=.7,zorder=8)
    ax.annotate('CoM',q[:2],xytext=(.55,-.40),fontsize=8.2,color=text_color,ha='center',
                arrowprops=dict(arrowstyle='-',color=ORANGE,lw=.8))
    ax.set(xlim=(-.85,.85),ylim=(-.5,.85),aspect='equal')
    # Measured preparation tensions, not a stylized signal.
    ax=fig.add_axes([.795,.752,.164,.137])
    s=data['time'];mask=(s>=0)&(s<=16)
    for i in range(4):ax.plot(s[mask],data['tension'][mask,i],color=PALETTE[i],lw=.65)
    ax.set(xlim=(0,16),xticks=[0,8,16],yticks=[2,4],xlabel='Time (s)',ylabel='Tension (N)')
    ax.tick_params(labelsize=7,pad=1.5,length=2.5,colors=text_color)
    ax.xaxis.label.set_size(7.5);ax.yaxis.label.set_size(7.5)
    ax.spines[['top','right']].set_visible(False);ax.grid(alpha=.12)
    fig.text(.550,.687,'Cable tensions and payload motion',fontsize=8.3,color=text_color)
    fig.text(.550,.640,r'$A\theta=b,\quad \theta=B\mu,\quad\mu\geq0$',fontsize=10,color=text_color)
    fig.text(.550,.592,r'$\hat m,\quad\hat c,\quad\hat J_O$',fontsize=10,color=ORANGE)
    fig.text(.750,.595,'Mass, CoM, inertia',fontsize=8.1,color=text_color)
    # (c) Two actual support configurations from the same course.
    for x,event,label in [(.551,events[0],'Side-on formation'),(.767,events[1],'Spread formation')]:
        ax=plain(fig,[x,.169,.195,.203]);configuration(ax,data,event['source_record_index'])
        fig.text(x+.0975,.143,label,fontsize=8.1,color=text_color,ha='center')
    fig.text(.550,.102,r'$G(R_r)f=Y_d\hat\theta\quad\longrightarrow\quad x_{i,r}(t)$',fontsize=10,color=text_color)
    fig.text(.550,.062,'Obstacle, separation and actuator constraints',fontsize=8,color=text_color)
    # (d) A genuine execution image beside the feedback-control signal chain.
    path=unreal/'inspection_views_clear/low_roof.png'
    ax=fig.add_axes([.036,.154,.226,.220]);ax.imshow(Image.open(path).crop((490,180,1550,1030)));ax.set_axis_off()
    steps=[(.308,'Payload pose\ncontrol'),(.230,'UAV position\nand attitude control'),(.152,'Rotor and cable\ndynamics')]
    for y,label in steps:
        canvas.add_patch(Rectangle((.282,y),.167,.060,facecolor='#F3F6F7',edgecolor=LIGHT,lw=.7))
        fig.text(.3655,y+.03,label,ha='center',va='center',fontsize=8,color=text_color,linespacing=1.1)
    for y in [.300,.222]:arrow(canvas,(.3655,y),(.3655,y-.010),color=TEAL,scale=7,lw=.8)
    arrow(canvas,(.281,.181),(.265,.181),color=TEAL,scale=8)
    fig.text(.039,.102,'50 Hz control; 10 Hz RGB-D requests',fontsize=8.2,color=text_color)
    fig.text(.039,.062,'Payload pose and cable tension feedback',fontsize=8.2,color=text_color)
    # Main chain and execution feedback. Phase tags prevent an online-planning inference.
    arrow(canvas,(.468,.765),(.531,.765),color=INK,scale=10)
    arrow(canvas,(.757,.544),(.757,.464),color=ORANGE,scale=10)
    fig.text(.772,.499,'Initial plan',fontsize=8,color=ORANGE,va='center')
    arrow(canvas,(.531,.250),(.469,.250),color=INK,scale=10)
    arrow(canvas,(.242,.464),(.242,.544),color=TEAL,scale=10,linestyle=(0,(3,2)))
    fig.text(.033,.499,'Visual and tension feedback',fontsize=8,color=TEAL,va='center')
    bind(folder/'scan_world.npy','Fig. 2 actual forked-payload measured scan')
    bind(folder/'IDENTIFICATION.json','Fig. 2 fitted moments and estimated CoM, no density reconstruction')
    bind(path,'Fig. 2 recorded Unreal execution view; spatial crop only')
    bind(folder/'ASSET.json','Fig. 2 nonconvex load facets and calibrated mounts')
    bind(folder/'trajectory.npz','Fig. 2 measured preparation tensions and actual support geometries')
