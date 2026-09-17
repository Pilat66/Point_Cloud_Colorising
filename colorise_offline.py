#!/usr/bin/env python3
"""Offline point-cloud colourisation from timestamped photos + a lidar trajectory."""
from __future__ import annotations
import argparse, json, math, os, sys, time as _time
from typing import Optional
import cv2
import numpy as np
try:
    from scipy.spatial import cKDTree
except Exception:
    cKDTree = None
try:
    import laspy
except Exception:
    laspy = None

def quat_normalize(q):
    n = math.sqrt(sum(v*v for v in q))
    return (1.0,0.0,0.0,0.0) if n < 1e-12 else tuple(v/n for v in q)
def quat_to_rot(q):
    w,x,y,z = quat_normalize(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
                     [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])
def quat_from_rpy(roll,pitch,yaw,order="xyz"):
    order=(order or "xyz").lower()
    cr,sr=math.cos(roll*.5),math.sin(roll*.5)
    cp,sp=math.cos(pitch*.5),math.sin(pitch*.5)
    cy,sy=math.cos(yaw*.5),math.sin(yaw*.5)
    if order=="xyz":
        return quat_normalize((cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy))
    if order=="zyx":
        return quat_normalize((cr*cp*cy+sr*sp*sy, cr*sp*cy-sr*cp*sy, cr*cp*sy-sr*sp*cy, sr*cp*cy+cr*sp*sy))
    raise ValueError("unknown --euler-order '{}'".format(order))
def quat_slerp(a,b,t):
    a=quat_normalize(a); b=quat_normalize(b)
    d=sum(x*y for x,y in zip(a,b))
    if d<0: b=tuple(-v for v in b); d=-d
    if 1.0-d<1e-6: return quat_normalize(tuple((1-t)*x+t*y for x,y in zip(a,b)))
    th=math.acos(max(-1.,min(1.,d))); st=math.sin(th)
    return quat_normalize(tuple(math.sin((1-t)*th)/st*x + math.sin(t*th)/st*y for x,y in zip(a,b)))
def make_Tmat(r,t):
    T=np.eye(4); T[:3,:3]=r; T[:3,3]=np.asarray(t,float).reshape(3); return T
def invert_T(T):
    out=np.eye(4); R=T[:3,:3]; out[:3,:3]=R.T; out[:3,3]=-(R.T@T[:3,3]); return out

def parse_timestamp_number(value):
    a=abs(value)
    if a>=1e15: return value/1e9
    if a>=1e11: return value/1e3
    return float(value)
def photo_timestamp_from_name(stem):
    s=stem.strip()
    if not s: return None
    if s.isdigit(): return parse_timestamp_number(int(s))
    runs,cur=[],""
    for ch in s:
        if ch.isdigit(): cur+=ch
        elif cur: runs.append(cur); cur=""
    if cur: runs.append(cur)
    return None if not runs else parse_timestamp_number(int(max(runs,key=len)))

class Trajectory:
    def __init__(self,times,pos,quats,columns):
        self.times=np.asarray(times,np.float64); self.pos=np.asarray(pos,np.float64)
        self.quats=np.asarray(quats,np.float64); self.columns=columns
        self.min_t=float(self.times.min()); self.max_t=float(self.times.max())
    def size(self): return int(self.times.shape[0])
    def sample_at(self,t):
        ts=self.times
        if t<ts[0] or t>ts[-1]: return self.pos[0],self.quats[0],False
        i=int(np.searchsorted(ts,t,side="right"))
        if i==0: return self.pos[0],self.quats[0],True
        if i>=len(ts): return self.pos[-1],self.quats[-1],True
        lo,hi=i-1,i; dt=ts[hi]-ts[lo]
        a=0.0 if dt<=1e-12 else (t-ts[lo])/dt
        return ((1-a)*self.pos[lo]+a*self.pos[hi], quat_slerp(self.quats[lo],self.quats[hi],a), True)
    def lidar_pose_world(self,t):
        p,q,ok=self.sample_at(t)
        return None if not ok else make_Tmat(quat_to_rot(q),p)
def _detect_units(vals):
    if vals.size==0: return "auto"
    nz=np.abs(vals)[np.abs(vals)>1e-6]
    if nz.size==0: return "auto"
    return "rad" if np.percentile(nz,90)<math.pi else "deg"
def load_trajectory(path,euler_order="xyz",euler_units="auto",time_shift=0.0):
    raw=open(path).read().splitlines()
    if not raw: raise ValueError("empty trajectory: "+path)
    first=raw[0]; delim=";" if first.count(";")>=first.count(",") else ","
    cols=[c.strip() for c in first.split(delim)]
    start,header=0,None
    if cols and cols[0].lower() in ("time","stamp","t","timestamp"): start,header=1,cols
    times,pos,quats=[],[],[]
    for line in raw[start:]:
        line=line.strip()
        if not line or line.startswith("#"): continue
        try: v=[float(x) for x in line.split(delim)]
        except ValueError: continue
        if len(v)>=8 and not header:
            q=quat_normalize((v[7],v[4],v[5],v[6])); times.append(v[0]+time_shift); pos.append(v[1:4]); quats.append(q)
        elif len(v)>=7:
            r,p,y=v[4],v[5],v[6]; u=euler_units
            if u=="auto":
                lu=_detect_units(np.array([r,p,y])); u=lu if lu!="auto" else "deg"
            if u=="deg": r,p,y=math.radians(r),math.radians(p),math.radians(y)
            times.append(v[0]+time_shift); pos.append(v[1:4]); quats.append(quat_from_rpy(r,p,y,euler_order))
    if len(times)<2: raise ValueError("trajectory has fewer than 2 usable rows ("+str(len(times))+")")
    o=np.argsort(np.asarray(times))
    return Trajectory(np.asarray(times)[o],np.asarray(pos)[o],np.asarray(quats)[o],header or [])

def _object_blocks(data):
    out=[]
    def walk(node,prefix):
        if isinstance(node,dict):
            p=prefix.rstrip(".")
            if p: out.append((p,node))
            for k,v in node.items(): walk(v,prefix+str(k)+".")
    walk(data,"")
    return out
def _cam_rank(path):
    last=str(path).split(".")[-1].lower()
    if last=="camera": return 3
    if last.startswith("camera"): return 2
    if last.startswith("cam"): return 1
    return 0
def _has_intrinsics(b):
    return isinstance(b,dict) and any(k in b for k in ("intrinsics","K","camera_matrix","fx","cam_fx"))
def _num_by_keys(obj,keys):
    for k in keys:
        v=obj.get(k)
        if isinstance(v,(int,float)) and not isinstance(v,bool): return float(v)
    return None
def load_camera_from_calib(path):
    """Camera params from the "camera" block of calib.json (the same file as the
    extrinsic): model, width/height, intrinsics and distortion coefficients.
    Returns (cam, key_used); key_used is the dotted path of the block read."""
    data=json.load(open(path))
    cand=[(p,b) for p,b in _object_blocks(data) if _has_intrinsics(b)]
    if not cand:
        raise ValueError("no camera intrinsics block in {} (expected {{\"camera\": {{\"width\": .., "
                         "\"height\": .., \"intrinsics\": [fx,fy,cx,cy], \"distortion_coeffs\": [..]}}}}, "
                         "see data/calib-pinhole-2026-09-16.json)".format(path))
    key,blk=sorted(cand,key=lambda x:(-_cam_rank(x[0]),x[0]))[0]
    model=str(blk.get("camera_model",blk.get("model",blk.get("distortion_model",
              blk.get("cam_model","PinholeCamera"))))).lower()
    cam={"model":"fisheye" if ("fish" in model or "equidistant" in model) else "pinhole"}
    w=_num_by_keys(blk,("width","image_width","cam_width"))
    h=_num_by_keys(blk,("height","image_height","cam_height"))
    if w is None or h is None:
        raise ValueError("camera block '{}' in {} has no image size: add \"width\" and \"height\" "
                         "(see data/calib-pinhole-2026-09-16.json)".format(key,path))
    cam["width"],cam["height"]=int(w),int(h)
    if "intrinsics" in blk:
        v=np.asarray(blk["intrinsics"],float).ravel()
        if v.size<4: raise ValueError("camera block '{}' in {}: 'intrinsics' must be [fx,fy,cx,cy]".format(key,path))
        cam["K"]=np.array([[v[0],0.0,v[2]],[0.0,v[1],v[3]],[0.0,0.0,1.0]])
    elif "K" in blk or "camera_matrix" in blk:
        v=np.asarray(blk["K"] if "K" in blk else blk["camera_matrix"],float).ravel()
        if v.size!=9: raise ValueError("camera block '{}' in {}: 'K' must be a 3x3 matrix".format(key,path))
        cam["K"]=v.reshape(3,3)
    else:
        fx,fy,cx,cy=(_num_by_keys(blk,ks) for ks in (("fx","cam_fx"),("fy","cam_fy"),("cx","cam_cx"),("cy","cam_cy")))
        if None in (fx,fy,cx,cy):
            raise ValueError("camera block '{}' in {}: no intrinsics "
                             "(expected \"intrinsics\": [fx,fy,cx,cy])".format(key,path))
        cam["K"]=np.array([[fx,0.0,cx],[0.0,fy,cy],[0.0,0.0,1.0]])
    if "distortion_coeffs" in blk: dists=blk["distortion_coeffs"]
    elif "dist_coeffs" in blk: dists=blk["dist_coeffs"]
    elif "D" in blk: dists=blk["D"]
    else: dists=[blk[k] for k in ("cam_d0","cam_d1","cam_d2","cam_d3","cam_d4","cam_d5") if blk.get(k) is not None]
    cam["dist"]=np.asarray(dists,float).ravel().astype(np.float64)
    return cam,key

def _norm7(v):
    x,y,z,qx,qy,qz,qw=[float(x) for x in v]
    return np.array([x,y,z]),(qw,qx,qy,qz)
def extrinsic_from_block(block,key):
    lower=key.lower().replace(" ","")
    lidar_from_cam=("lidar_camera" in lower) or ("lidar_from_cam" in lower)
    if isinstance(block,dict):
        if "translation" in block or "t" in block:
            t=np.array(block.get("translation",block.get("t")),float)
            if "quaternion" in block:
                q=block["quaternion"]
                q=_norm7([0,0,0]+list(q))[1] if len(q)==7 else (q[0],q[1],q[2],q[3])
                R=quat_to_rot(q)
            elif "rotation" in block: R=np.array(block["rotation"],float).reshape(3,3)
            elif "R" in block: R=np.array(block["R"],float).reshape(3,3)
            else: raise ValueError("extrinsic '{}': no rotation/quaternion".format(key))
            T=make_Tmat(R,t)
        elif block.get("matrix") is not None: T=np.array(block["matrix"],float)
        else: raise ValueError("extrinsic '{}': unknown object layout".format(key))
    else:
        arr=np.asarray(block,float).ravel()
        if arr.size==7:
            pos,q=_norm7(arr); T=make_Tmat(quat_to_rot(q),pos)
        elif arr.size==16: T=np.array(block,float).reshape(4,4)
        elif arr.size==12:
            T=np.eye(4); T[:3,:3]=arr[:9].reshape(3,3); T[:3,3]=arr[9:12]
        else: raise ValueError("extrinsic '{}': cannot interpret {} numbers".format(key,arr.size))
    T=np.asarray(T,float).reshape(4,4)
    if lidar_from_cam: T=invert_T(T)
    return T
def load_calib(path,key=None,direction=None):
    data=json.load(open(path)); blocks=[]
    def collect(node,prefix):
        if isinstance(node,dict):
            for k,v in node.items(): collect(v,prefix+str(k)+".")
        else: blocks.append((prefix.rstrip("."),node))
    collect(data,"")
    named=[(f,b) for f,b in blocks if f.split(".")[-1] in ("T_camera_lidar","T_lidar_camera","T_lidar_cam","T_cam_lidar")]
    if key:
        sel=next(((f,b) for f,b in blocks if f==key or f.endswith(key)),None)
        if sel is None: raise KeyError("extrinsic key '{}' not found in {}".format(key,path))
        full=key
    elif named:
        full,blk=named[0]; sel=(full,blk)
    else:
        cand=[x for x in blocks if isinstance(x[1],dict) or len(np.asarray(x[1],float).ravel()) in (4,7,12,16)]
        if not cand: raise ValueError("no extrinsic-like block found in "+path)
        sel=max(cand,key=lambda x: len(str(x[1]))); full=sel[0]
    T=extrinsic_from_block(sel[1],full)
    if direction=="lidar_from_camera": T=invert_T(T)
    return T, full, direction or ("auto")

def _pcd_header(path):
    hdr={}; off=0
    with open(path,"rb") as f:
        while True:
            line=f.readline()
            if not line: break
            s=line.decode("ascii","replace").rstrip("\r\n")
            if not s.strip() or s.startswith("#"): continue
            parts=s.split()
            if parts[0]=="DATA": hdr["DATA"]=parts[1] if len(parts)>1 else "binary"; off=f.tell(); break
            hdr[parts[0]]=parts[1:]
    return hdr,off
def load_cloud(path):
    ext=os.path.splitext(path)[1].lower()
    if ext==".las":
        if laspy is None: raise RuntimeError("install laspy for .las input, or use .pcd")
        las=laspy.read(path)
        xyz=np.column_stack([las.x,las.y,las.z]).astype(np.float64)
        inten=np.asarray(las.intensity,np.float64) if hasattr(las,"intensity") else None
        return xyz,inten
    if ext!=".pcd": raise ValueError("unsupported cloud: "+ext)
    hdr,off=_pcd_header(path)
    fields=hdr["FIELDS"]; sizes=[int(x) for x in hdr["SIZE"]]; npts=int(hdr["POINTS"][0])
    dt=np.dtype([(f,"<f4") if hdr["TYPE"][i]=="F" else (f,"<u4" if hdr["TYPE"][i]=="U" else "<i4") for i,f in enumerate(fields)])
    with open(path,"rb") as f:
        f.seek(off); data=np.frombuffer(f.read(npts*dt.itemsize),dtype=dt,count=npts)
    xyz=np.column_stack([data["x"],data["y"],data["z"]]).astype(np.float64)
    inten=data["intensity"].astype(np.float64) if "intensity" in fields else None
    return xyz,inten
def save_cloud_pcd(path,xyz,rgb,intensity):
    n=xyz.shape[0]; rgb=np.asarray(rgb,np.uint32)
    if intensity is None: intensity=np.zeros(n,np.float32)
    dt=np.dtype([("x","<f4"),("y","<f4"),("z","<f4"),("intensity","<f4"),("rgb","<u4")])
    arr=np.empty(n,dtype=dt); arr["x"]=xyz[:,0]; arr["y"]=xyz[:,1]; arr["z"]=xyz[:,2]
    arr["intensity"]=intensity; arr["rgb"]=rgb
    with open(path,"wb") as f:
        hdr=("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity rgb\n"
             "SIZE 4 4 4 4 4\nTYPE F F F F U\nCOUNT 1 1 1 1 1\nWIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA binary\n"%(n,n))
        f.write(hdr.encode("ascii")); f.write(arr.tobytes())
def pack_rgb(r,g,b):
    r=np.asarray(r,np.uint32); g=np.asarray(g,np.uint32); b=np.asarray(b,np.uint32)
    return (r<<16)|(g<<8)|b
def unpack_rgb(p):
    p=np.asarray(p,np.uint32); return np.stack([(p>>16)&255,(p>>8)&255,p&255],axis=1)

def project_and_sample(P,idx,img,K,dist,model,cam_w,cam_h,T_cam_from_pts,edge_margin=0.0,max_view_angle_deg=180.0,min_camera_dist=0.0,occlusion=True,occl_cell=4.0,occl_tol=0.3):
    if idx is None or len(idx)==0 or img is None:
        return np.zeros(0,np.uint32),np.zeros(0,np.int64),np.zeros(0,np.float64)
    H,W=img.shape[:2]
    R=T_cam_from_pts[:3,:3]; t=T_cam_from_pts[:3,3]
    Pc=P[idx]@R.T+t; z=Pc[:,2]; d3=np.linalg.norm(Pc,axis=1)
    ok=(z>1e-6)&(d3>1e-6)
    if min_camera_dist>0: ok&=d3>=min_camera_dist
    if max_view_angle_deg<180.0: ok&=(z/d3)>=np.cos(math.radians(max_view_angle_deg))
    ii=idx[ok]; pc=Pc[ok]
    if pc.shape[0]==0: return np.zeros(0,np.uint32),np.zeros(0,np.int64),np.zeros(0,np.float64)
    rv=np.zeros((3,1)); tv=np.zeros((3,1))
    if model=="fisheye" and hasattr(cv2,"fisheye"):
        _r=cv2.fisheye.projectPoints(pc.astype(np.float32),rv,tv,K,dist); P2=np.asarray(_r[0] if isinstance(_r,tuple) else _r).reshape(-1,2)
    else:
        _r=cv2.projectPoints(pc.astype(np.float32),rv,tv,K,dist); P2=np.asarray(_r[0] if isinstance(_r,tuple) else _r).reshape(-1,2)
    fin=np.isfinite(P2[:,0])&np.isfinite(P2[:,1])
    P2s=np.nan_to_num(P2, nan=-1e9, posinf=-1e9, neginf=-1e9)
    u=np.round(P2s[:,0]).astype(np.int64)
    v=np.round(P2s[:,1]).astype(np.int64)
    m=int(round(edge_margin))
    inside=fin&(u>=m)&(u<cam_w-m)&(v>=m)&(v<cam_h-m)
    ii,pc,u,v,depth=ii[inside],pc[inside],u[inside],v[inside],pc[inside,2]
    # camera-to-point distance for --nearest-wins: батчевое приближение sqrt
    # (бит-трюк + одна итерация Ньютона, ошибка <0.2%); арифметика бит-в-бит
    # совпадает с fastSqrtApprox() в colorise/src/project.cpp — паритет C++/Python.
    d2=(pc*pc).sum(1)
    f=d2.astype(np.float32)
    y=(np.uint32(0x5f3759df)-(f.view(np.uint32)>>np.uint32(1))).view(np.float32)
    y=y*(np.float32(1.5)-np.float32(0.5)*f*y*y)
    dnorm=(f*y).astype(np.float64)
    if pc.shape[0]==0:
        return np.zeros(0,np.uint32),np.zeros(0,np.int64),np.zeros(0,np.float64)
    if occlusion:
        cell=max(1.0,occl_cell); gx=(u/cell).astype(np.int64); gy=(v/cell).astype(np.int64)
        gw=int(math.ceil(cam_w/cell)); zmin=np.full(int(math.ceil(cam_h/cell))*gw,np.inf)
        np.minimum.at(zmin,gy*gw+gx,depth.astype(np.float64))
        keep=depth<=(zmin[gy*gw+gx]+occl_tol); ii,u,v,dnorm=ii[keep],u[keep],v[keep],dnorm[keep]
        if ii.shape[0]==0:
            return np.zeros(0,np.uint32),np.zeros(0,np.int64),np.zeros(0,np.float64)
    if W!=cam_w or H!=cam_h:
        u=np.clip((u*(W/cam_w)).astype(np.int64),0,W-1); v=np.clip((v*(H/cam_h)).astype(np.int64),0,H-1)
    else:
        u=np.clip(u,0,W-1); v=np.clip(v,0,H-1)
    px=img[v,u]; return pack_rgb(px[:,2],px[:,1],px[:,0]),ii,dnorm

def colourise(args):
    cam,cam_key=load_camera_from_calib(args.calib)
    print("[camera] {} {}x{} (calib: {}) K={} dist={}".format(cam['model'],cam['width'],cam['height'],cam_key,np.round(cam['K'],3).tolist(),np.round(cam['dist'],5).tolist()))
    T,key,direction=load_calib(args.calib,args.extrinsic_name,args.extrinsic_direction)
    print("[extrinsic] {} ({})\n{}".format(key,direction,np.round(T,4)))
    tr=load_trajectory(args.trajectory,args.euler_order,args.euler_units,args.time_shift)
    print("[trajectory] {} poses t=[{:.3f},{:.3f}]".format(tr.size(),tr.min_t,tr.max_t))
    exts=(".jpg",".jpeg",".png",".bmp")
    ph=[]
    for fn in sorted(os.listdir(args.photos)):
        if os.path.splitext(fn)[1].lower() not in exts: continue
        ts=photo_timestamp_from_name(os.path.splitext(fn)[0])
        if ts is not None: ph.append((ts,os.path.join(args.photos,fn)))
    ph.sort()
    if not ph: raise RuntimeError("no timestamped images in "+args.photos)
    print("[photos] {} t=[{:.3f},{:.3f}]".format(len(ph),ph[0][0],ph[-1][0]))
    if ph[0][0]>tr.max_t or ph[-1][0]<tr.min_t:
        print("[WARN] photo times and trajectory do not overlap -> colourisation will cover nothing; align with --time-shift or use matching data")
    t0=_time.time(); xyz,intensity=load_cloud(args.cloud)
    print("[cloud] loading {:,} pts in {:.1f}s".format(xyz.shape[0],_time.time()-t0))
    fin=np.isfinite(xyz).all(1)
    if args.max_lidar_z<np.inf: fin&=xyz[:,2]<=args.max_lidar_z
    xyz=xyz[fin]
    if intensity is not None: intensity=intensity[fin]
    n=xyz.shape[0]
    cull=cKDTree(xyz) if (args.max_range>0 and cKDTree is not None) else None
    csum=np.zeros((n,3)); ccnt=np.zeros(n,np.int32)
    out=np.zeros(n,np.uint32); colored=np.zeros(n,bool)
    best=np.full(n,np.inf)   # --nearest-wins: dist of the camera that coloured the point
    def maybe_T(pt,tol=0.0):
        # Anchor to the nearest endpoint pose when the photo is only slightly
        # outside the trajectory span (within tol); otherwise report 'no pose'.
        if pt<tr.min_t-tol or pt>tr.max_t+tol:
            return None
        tc=max(tr.min_t,min(tr.max_t,pt))
        Tw=tr.lidar_pose_world(tc)
        return None if Tw is None else invert_T(Tw@invert_T(T))
    for k,(pt,pfile) in enumerate(ph,1):
        Tc=maybe_T(pt,args.time_tolerance)
        if Tc is None:
            print("[frame {}] t={:.3f} outside trajectory span (tol {:.2f}s) -> skipped".format(k,pt,args.time_tolerance)); continue
        camw=(np.linalg.inv(Tc)@np.array([0,0,0,1.0]))[:3]
        if cull is not None:
            ids=cull.query_ball_point(camw,args.max_range)   # flat list for a single query point
            cand=np.asarray(ids,np.int64) if len(ids) else np.array([],np.int64)
        else:
            cand=np.arange(n)
        if cand.size==0:
            print("[frame {}] no points within {:.0f} m -> skipped".format(k,args.max_range)); continue
        img=cv2.imread(pfile,cv2.IMREAD_COLOR)
        if img is None: continue
        rgb,ii,dist=project_and_sample(xyz,cand,img,cam['K'],cam['dist'],cam['model'],cam['width'],cam['height'],Tc,
                edge_margin=args.edge_margin,max_view_angle_deg=args.max_view_angle,min_camera_dist=args.min_camera_dist,
                occlusion=args.occlusion,occl_cell=args.occlusion_cell,occl_tol=args.occlusion_depth_tol)
        if ii.size:
            if args.first_wins:
                m=colored[ii]==0
                if m.any():
                    nz=ii[m]; px=unpack_rgb(rgb)[m]
                    out[nz]=pack_rgb(px[:,0],px[:,1],px[:,2]); colored[nz]=True
            elif args.nearest_wins:
                # keep the observation from the nearest camera; strict '<' makes
                # the result independent of the frame order / threading
                m=dist<best[ii]
                if m.any():
                    nz=ii[m]; out[nz]=rgb[m]; best[nz]=dist[m]; colored[nz]=True
                ccnt[ii]+=1
            else:
                px=unpack_rgb(rgb); np.add.at(csum,ii,px); ccnt[ii]+=1
            uniq_so_far = int(colored.sum()) if (args.first_wins or args.nearest_wins) else int((ccnt>0).sum())
            print("[frame {}] t={:.3f} coloured={:,} cand={:,} uniq_so_far={:,}".format(k,pt,ii.size,cand.size,uniq_so_far))
    if args.first_wins:
        has=colored
    elif args.nearest_wins:
        has=ccnt>=args.min_color_frames   # out already holds the nearest-camera colour
    else:
        has=ccnt>=args.min_color_frames
        if has.any():
            av=csum[has]/ccnt[has,None]; out[has]=pack_rgb(av[:,0].round(),av[:,1].round(),av[:,2].round())
    sel=has if not args.keep_uncolored else np.ones(n,bool)
    max_obs=int(ccnt.max()) if n else 0
    if int(has.sum())==0 and max_obs>0 and args.min_color_frames>1:
        print("\n[WARN] frames coloured points (max {} observations/point) but nothing kept: "
              "--min-color-frames={} is too high for this run. Try --min-color-frames 1."
              .format(max_obs,args.min_color_frames))
    print("\n==== RESULT ===="); print("  total={:,}  coloured={:,}  ({:.2f}%)".format(n,int(has.sum()),100.0*has.mean()))
    if args.output:
        save_cloud_pcd(args.output,xyz[sel],out[sel],None if intensity is None else intensity[sel])
        print("  wrote {}".format(args.output))
    return {"total":n,"coloured":int(has.sum())}

def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cloud",default="data/all_raw_points.pcd")
    ap.add_argument("--photos",default="data")
    ap.add_argument("--trajectory",default="data/trajectory.csv")
    ap.add_argument("--calib",default="data/calib.json",
                    help="camera intrinsics (\"camera\" block) + camera<->lidar extrinsic")
    ap.add_argument("--output",default="coloured.pcd")
    ap.add_argument("--preview",default="")
    ap.add_argument("--extrinsic-name",default=None)
    ap.add_argument("--extrinsic-direction",choices=[None,"camera_from_lidar","lidar_from_camera"],default=None)
    ap.add_argument("--euler-order",default="xyz",choices=["xyz","zyx"])
    ap.add_argument("--euler-units",default="auto",choices=["auto","deg","rad"])
    ap.add_argument("--time-shift",type=float,default=0.0)
    ap.add_argument("--time-tolerance",type=float,default=1.0,
                    help="s; allow a photo up to this much outside the trajectory span "
                         "(pose clamped to the nearest trajectory endpoint)")
    ap.add_argument("--max-range",type=float,default=20.0)
    ap.add_argument("--min-camera-dist",type=float,default=2.0)
    ap.add_argument("--max-view-angle",type=float,default=75.0)
    ap.add_argument("--edge-margin",type=float,default=0.0)
    ap.add_argument("--occlusion",action="store_true",default=True)
    ap.add_argument("--occlusion-cell",type=float,default=4.0)
    ap.add_argument("--occlusion-depth-tol",type=float,default=0.3)
    ap.add_argument("--min-color-frames",type=int,default=1)
    ap.add_argument("--keep-uncolored",action="store_true")
    ap.add_argument("--first-wins",action="store_true",help="keep the first colour a point receives instead of averaging across frames")
    ap.add_argument("--nearest-wins",action="store_true",help="colour each point only from the nearest camera (min camera-to-point distance among frames where the point is visible)")
    ap.add_argument("--max-lidar-z",type=float,default=np.inf)
    args=ap.parse_args(argv)
    if args.first_wins and args.nearest_wins:
        raise SystemExit("--first-wins и --nearest-wins взаимоисключающие: выберите один режим")
    if args.output:
        import os as _os; _os.makedirs(_os.path.dirname(_os.path.abspath(args.output)),exist_ok=True)
    try:
        colourise(args)
    except Exception as e:
        print("\n[error] {}: {}".format(type(e).__name__,e),file=sys.stderr); return 1
    return 0
if __name__=="__main__":
    raise SystemExit(main())
