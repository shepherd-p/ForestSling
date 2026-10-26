"""Corner pad-eyes and articulated cable terminals, in payload NEU metres."""
import numpy as np


def corner_rigging(cells,half,count=4):
    cells=np.asarray(cells);half=np.asarray(half)
    if count not in (3,4):raise ValueError('Three or four corner mounts required')
    lo,hi=cells.min(0)-half,cells.max(0)+half;mounts=[]
    for side,x in ((-1,lo[0]+.05),(1,hi[0]-.05)):
        column=cells[np.abs(cells[:,0]-x)<=half[0]+1e-8]
        ymin,ymax=column[:,1].min()-half[1],column[:,1].max()+half[1]
        ys=[ymin+.045,ymax-.045] if side<0 or count==4 else [(ymin+ymax)/2]
        for y in ys:
            support=cells[(np.abs(cells[:,0]-x)<=half[0]+1e-8)&(np.abs(cells[:,1]-y)<=half[1]+1e-8)]
            if not len(support):raise ValueError('Corner mount has no material support')
            foot=np.array([x,y,support[:,2].max()+half[2]])
            radial=np.array([side,np.sign(y-(ymin+ymax)/2),0.]);radial/=np.linalg.norm(radial)
            normal=np.cross([0,0,1.],radial)
            radius=.024;tube=.004;plate_half=np.array([.025,.025,.0015]);pin=.0035
            center=foot+[0,0,2*plate_half[2]+radius]
            # The pin bears against the upper inner surface of the pad-eye.
            anchor=center+[0,0,radius-tube-pin]
            mounts.append(dict(foot=foot,radial=radial,eye_normal=normal,eye_center=center,
                eye_radius=radius,eye_tube_radius=tube,plate_half=plate_half,
                anchor=anchor,connector_length=.060,connector_pin_radius=pin))
    return dict(type='corner_padeyes',mounts=mounts,
        cable_model='Massless rigid terminal included in the effective 2 m cable length; force acts at the lower pin centre')


def eye_points(mount,segments=24):
    t=np.linspace(0,2*np.pi,segments+1)
    return np.asarray(mount['eye_center'])+mount['eye_radius']*(np.cos(t)[:,None]*np.asarray(mount['radial'])+np.sin(t)[:,None]*[0,0,1.])


def fixed_mass_parts(rigging,density=2700.):
    parts=[]
    for m in rigging['mounts']:
        h=np.asarray(m['plate_half']);mass=density*8*np.prod(h)
        parts.append((mass,np.asarray(m['foot'])+[0,0,h[2]],mass/3*np.diag(np.sum(h*h)-h*h)))
        r,R=m['eye_tube_radius'],m['eye_radius'];mass=density*2*np.pi**2*R*r*r;n=np.asarray(m['eye_normal'])
        inertia=mass*((.5*R*R+.625*r*r)*np.eye(3)+(.5*R*R+.125*r*r)*np.outer(n,n))
        parts.append((mass,np.asarray(m['eye_center']),inertia))
    return parts


class Mesh:
    def __init__(self):self.vertices=[];self.normals=[];self.uv=[];self.indices=[]
    def quad(self,points,outward=None):
        points=np.asarray(points);normal=np.cross(points[1]-points[0],points[2]-points[0]);normal/=max(1e-12,np.linalg.norm(normal))
        if outward is not None and normal@outward<0:points=points[::-1];normal=-normal
        k=len(self.vertices);self.vertices.extend(points);self.normals.extend([normal]*4);self.uv.extend([[0,0],[1,0],[1,1],[0,1]])
        self.indices.extend((k+np.array([0,1,2,0,2,3])).tolist())
    def box(self,center,half):
        center=np.asarray(center);half=np.asarray(half)
        for axis in range(3):
            others=[i for i in range(3) if i!=axis]
            for sign in (-1,1):
                c=center+sign*half[axis]*np.eye(3)[axis]
                self.quad([c+u*half[others[0]]*np.eye(3)[others[0]]+v*half[others[1]]*np.eye(3)[others[1]] for u,v in [(-1,-1),(1,-1),(1,1),(-1,1)]],sign*np.eye(3)[axis])
    def tube(self,points,radius,sides=10):
        points=np.asarray(points)
        for a,b in zip(points[:-1],points[1:]):
            z=b-a;z/=np.linalg.norm(z);x=np.cross(z,[0,0,1.])
            if np.linalg.norm(x)<.1:x=np.cross(z,[0,1.,0])
            x/=np.linalg.norm(x);y=np.cross(z,x);t=np.linspace(0,2*np.pi,sides+1)
            ring=radius*(np.cos(t)[:,None]*x+np.sin(t)[:,None]*y)
            for j in range(sides):self.quad([a+ring[j],b+ring[j],b+ring[j+1],a+ring[j+1]],ring[j]+ring[j+1])
    def asset(self,identity=2601,color=(215,165,49)):
        return dict(identity=identity,family='girder',colour=list(color),metallic=.65,roughness=.32,
            vertices=np.asarray(self.vertices)*[1,1,-1],normals=np.asarray(self.normals)*[1,1,-1],
            uv=np.asarray(self.uv),indices=np.asarray(self.indices))


def fixed_mesh(rigging):
    mesh=Mesh()
    for m in rigging['mounts']:
        h=np.asarray(m['plate_half']);mesh.box(np.asarray(m['foot'])+[0,0,h[2]],h)
        mesh.tube(eye_points(m,48),m['eye_tube_radius'])
    return mesh.asset()


def connector_mesh():
    # Local z follows the cable. The transverse pin passes through the fixed
    # eye; the U-bow and rope ferrule form a continuous visible load path.
    mesh=Mesh();w=.012;r=.0035
    mesh.tube([[-.019,0,0],[.019,0,0]],r)
    t=np.linspace(0,np.pi,17)
    path=np.vstack(([w,0,0],[w,0,.032],np.column_stack([w*np.cos(t),np.zeros_like(t),.032+w*np.sin(t)]),[-w,0,0]))
    # Repeated points at the arch junction are removed before tubing.
    path=path[np.r_[True,np.linalg.norm(np.diff(path,axis=0),axis=1)>1e-10]]
    mesh.tube(path,r);mesh.tube([[0,0,.043],[0,0,.072]],.0085)
    return mesh.asset(identity=2602,color=(190,196,204))


def connector_rotation(direction,eye_normal):
    z=np.asarray(direction,float);z/=np.linalg.norm(z)
    x=np.asarray(eye_normal,float)-z*float(np.asarray(eye_normal)@z)
    if np.linalg.norm(x)<1e-6:x=np.cross(z,[0,1.,0])
    x/=np.linalg.norm(x)
    return np.column_stack([x,np.cross(z,x),z])
