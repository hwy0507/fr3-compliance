"""Offline fusion of original camera frames and directly logged ESN states.
No robot/device/network access. Match original timestamps, never just frame indices.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
os.environ.setdefault('MPLCONFIGDIR','/tmp/fr3-neuron-matplotlib')
import cv2
import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from matplotlib import colormaps

def rows(path):
    with path.open() as f:return list(csv.DictReader(f))

def _mdh(a, alpha, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st, 0.0, a],
        [st * ca, ct * ca, -sa, -sa * d],
        [st * sa, ct * sa, ca, ca * d],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)

def fr3_fk_xyz(q, ee_z=0.255):
    """Compute FR3 end-effector (x, y, z) in meters from 7 joint angles using MDH."""
    # Standard Franka Research 3 / Panda modified DH parameters + flange (0.107 m) + F_T_EE z-offset.
    params = [
        (0.0, 0.0, 0.333, q[0]),
        (0.0, -np.pi / 2, 0.0, q[1]),
        (0.0, np.pi / 2, 0.316, q[2]),
        (0.0825, np.pi / 2, 0.0, q[3]),
        (-0.0825, -np.pi / 2, 0.384, q[4]),
        (0.0, np.pi / 2, 0.0, q[5]),
        (0.088, np.pi / 2, 0.0, q[6]),
    ]
    T = np.eye(4, dtype=float)
    for a, alpha, d, theta in params:
        T = T @ _mdh(a, alpha, d, theta)
    T = T @ np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.107 + ee_z],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)
    return T[:3, 3]

def load_ee_trajectory(policy_rows, diag_dir):
    """Load end-effector XYZ (meters) from callback_states.csv if present, else FK from obs0..6."""
    cb_path = Path(diag_dir) / 'callback_states.csv'
    if cb_path.exists():
        cb_rows = rows(cb_path)
        if cb_rows and 'O_T_EE_12' in cb_rows[0] and 'robot_time_s' in policy_rows[0]:
            cb_t = np.array([float(r['robot_time_s']) for r in cb_rows], dtype=float)
            cb_xyz = np.array([[float(r['O_T_EE_12']), float(r['O_T_EE_13']), float(r['O_T_EE_14'])] for r in cb_rows], dtype=float)
            p_t = np.array([float(r['robot_time_s']) for r in policy_rows], dtype=float)
            idx = np.clip(np.searchsorted(cb_t, p_t), 0, len(cb_t) - 1)
            prev = np.maximum(0, idx - 1)
            choose_prev = np.abs(cb_t[prev] - p_t) < np.abs(cb_t[idx] - p_t)
            nearest = np.where(choose_prev, prev, idx)
            return cb_xyz[nearest], 'callback_states_O_T_EE'
    qs = np.array([[float(r[f'obs{i}']) for i in range(7)] for r in policy_rows], dtype=float)
    return np.array([fr3_fk_xyz(q) for q in qs], dtype=float), 'policy_q_fk'

def load_reference_trajectory(diag_dir, ref_session=None, max_samples=400):
    """Load obstacle-free reference trajectory from --ref-session or nominal_reference in raw_commands.csv."""
    if ref_session is not None:
        ref_root = Path(ref_session).resolve()
        ref_cycle = next(ref_root.glob('cycle_*'))
        ref_policy = ref_cycle / 'execute_diagnostics/policy_sync.csv'
        ref_xyz, ref_src = load_ee_trajectory(rows(ref_policy), ref_policy.parent)
        return ref_xyz, f'ref_session:{ref_src}'
    raw_path = Path(diag_dir) / 'raw_commands.csv'
    if raw_path.exists():
        raw_rows = rows(raw_path)
        if raw_rows and 'nominal_reference_0' in raw_rows[0]:
            step = max(1, len(raw_rows) // max_samples)
            sampled = raw_rows[::step]
            if raw_rows[-1] is not sampled[-1]:
                sampled.append(raw_rows[-1])
            qs = np.array([[float(r[f'nominal_reference_{i}']) for i in range(7)] for r in sampled], dtype=float)
            return np.array([fr3_fk_xyz(q) for q in qs], dtype=float), 'nominal_reference_fk'
    return None, None

def draw_dashed_polyline(draw, pts, fill, width=2, dash_px=8.0, gap_px=6.0):
    """Draw a dashed polyline over a sequence of 2D pixel points."""
    if len(pts) < 2:
        return
    drawing = True
    rem = dash_px
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        dx, dy = float(x1 - x0), float(y1 - y0)
        seg_len = float(np.hypot(dx, dy))
        if seg_len < 1e-6:
            continue
        pos = 0.0
        while pos < seg_len:
            step = min(rem, seg_len - pos)
            t0, t1 = pos / seg_len, (pos + step) / seg_len
            if drawing:
                p0 = (int(round(x0 + dx * t0)), int(round(y0 + dy * t0)))
                p1 = (int(round(x0 + dx * t1)), int(round(y0 + dy * t1)))
                draw.line((p0, p1), fill=fill, width=width)
            pos += step
            rem -= step
            if rem <= 1e-6:
                drawing = not drawing
                rem = dash_px if drawing else gap_px

def project_xy_box(xyz_m, box, pad=0.12, extra_xyz_m=None):
    """Project XY coordinates (meters) into pixel coordinates inside box with equal aspect ratio."""
    x0, y0, x1, y1 = box
    all_xyz = np.vstack([xyz_m, extra_xyz_m]) if extra_xyz_m is not None else xyz_m
    all_mm = all_xyz[:, :2] * 1000.0
    xmin, ymin = all_mm.min(axis=0)
    xmax, ymax = all_mm.max(axis=0)
    span = max(float(xmax - xmin), float(ymax - ymin), 20.0)
    cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
    half = 0.5 * span * (1.0 + 2.0 * pad)
    w, h_box = x1 - x0, y1 - y0
    side = min(w, h_box)
    ox, oy = x0 + 0.5 * (w - side), y0 + 0.5 * (h_box - side)
    def _to_px(arr):
        return [
            (
                int(round(ox + (x - (cx - half)) / (2.0 * half) * side)),
                int(round(oy + ((cy + half) - y) / (2.0 * half) * side)),
            )
            for x, y in (arr[:, :2] * 1000.0)
        ]
    return _to_px(xyz_m), (cx - half, cx + half, cy - half, cy + half), (_to_px(extra_xyz_m) if extra_xyz_m is not None else None)

def _rodrigues(rvec):
    theta = float(np.linalg.norm(rvec))
    if theta < 1e-12:
        return np.eye(3, dtype=float)
    k = rvec / theta
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]], dtype=float)
    return np.eye(3, dtype=float) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)

def project_3d_to_camera(xyz_m, calib):
    """Project 3D base-frame points (N, 3) in meters to 2D camera pixels (u, v) and validity mask."""
    K = np.asarray(calib['camera_matrix'], dtype=float)
    dist = np.asarray(calib.get('dist_coeffs', [0.0, 0.0, 0.0, 0.0, 0.0]), dtype=float).ravel()
    rvec = np.asarray(calib['rvec'], dtype=float).ravel()
    tvec = np.asarray(calib['tvec'], dtype=float).ravel()
    w, h_img = calib.get('image_size', [640, 480])
    R = _rodrigues(rvec)
    pts_cam = (xyz_m @ R.T) + tvec[None, :]
    z = pts_cam[:, 2]
    in_front = z > 0.05
    safe_z = np.where(in_front, z, 1.0)
    x = pts_cam[:, 0] / safe_z
    y = pts_cam[:, 1] / safe_z
    if np.any(dist != 0.0):
        k1, k2, p1, p2, k3 = (list(dist) + [0.0] * 5)[:5]
        r2 = x * x + y * y
        radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    else:
        xd, yd = x, y
    u = K[0, 0] * xd + K[0, 1] * yd + K[0, 2]
    v = K[1, 1] * yd + K[1, 2]
    inside = in_front & (u >= 0.0) & (u < float(w)) & (v >= 0.0) & (v < float(h_img))
    return np.column_stack([u, v]), inside

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--session',type=Path,required=True)
    ap.add_argument('--camera-calib',type=Path,default=None,help='Optional camera_calib.json for 3D-to-2D end-effector trajectory overlay')
    ap.add_argument('--ref-session',type=Path,default=None,help='Optional obstacle-free session directory for reference trajectory comparison')
    args=ap.parse_args();root=args.session.resolve();out=root/'fusion';out.mkdir(exist_ok=False)
    camera=root/'camera';cycle=next(root.glob('cycle_*'));policy=cycle/'execute_diagnostics/policy_sync.csv'
    alignment=json.loads((root/'alignment/alignment.json').read_text())
    execution=json.loads((cycle/'execute_result.json').read_text())
    aborted=execution['return_code']!=0
    esn_enabled=json.loads((root/'session.json').read_text()).get('esn_enabled',True)
    pc=json.loads((policy.parent/'policy_clock.json').read_text())
    assert not pc['synthetic_test_data'] and pc['direct_reservoir_states']
    assert not json.loads((policy.parent/'policy_sync_status.json').read_text())['overflow']
    cr=rows(camera/'camera_frames.csv');pr=rows(policy);pairs=rows(root/'alignment/frame_policy_pairs.csv')
    assert len(cr)==len(pairs)==json.loads((camera/'complete.json').read_text())['frames']
    n=len(cr);framebytes=640*480*2
    assert (camera/'camera.yuyv').stat().st_size==n*framebytes
    raw=np.memmap(camera/'camera.yuyv',mode='r',dtype=np.uint8,shape=(n,480,640,2))
    ct=np.array([int(r['capture_monotonic_ns']) for r in cr],dtype=np.int64)
    pt=np.array([int(r['callback_monotonic_ns']) for r in pr],dtype=np.int64)
    h=np.array([[float(r[f'h{i}']) for i in range(128)] for r in pr])
    actions=np.array([[float(r[f'a{i}']) for i in range(7)] for r in pr])
    force=np.linalg.norm(np.array([[float(r[f'obs{i}']) for i in range(39,42)] for r in pr]),axis=1)
    ee_xyz,ee_source=load_ee_trajectory(pr,policy.parent)
    ref_xyz,ref_source=load_reference_trajectory(policy.parent,args.ref_session)
    ref_dev_mm=(np.linalg.norm(ee_xyz[:,None,:]-ref_xyz[None,:,:],axis=2).min(axis=1)*1000.0) if ref_xyz is not None else None
    calib_path=args.camera_calib or (root/'camera_calib.json' if (root/'camera_calib.json').exists() else None)
    calib=json.loads(Path(calib_path).read_text()) if calib_path else None
    cam_uv,cam_uv_ok=(project_3d_to_camera(ee_xyz,calib) if calib else (None,None))
    ref_cam_uv,ref_cam_ok=(project_3d_to_camera(ref_xyz,calib) if (calib and ref_xyz is not None) else (None,None))
    dh=np.sqrt(np.mean(np.diff(h,axis=0,prepend=np.zeros((1,128)))**2,axis=1))
    assert np.isfinite(h).all() and np.max(abs(h))<=1.000001
    valid=np.array([int(r['valid'])==1 for r in pairs]);sel=np.flatnonzero(valid)
    first,last=int(sel[0]),int(sel[-1]);start=int(ct[first]);end=int(ct[last]);fps=30
    timeline=start+np.rint(np.arange(int((end-start)/1e9*fps)+1)*1e9/fps).astype(np.int64)
    t=(pt-pt[0])/1e9;duration=float((end-pt[0])/1e9)
    contact=[];active=False;timer=0.
    for f in force:
        if not active:
            if f>12:active=True;timer=0.
        elif f<9:
            timer+=.04
            if timer>.3:active=False
        else:timer=0.
        contact.append(active)
    contact=np.array(contact)
    if not esn_enabled:contact[:]=False
    normal='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    bold='/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
    fonts={s:ImageFont.truetype(normal,s) for s in (13,15,17,19,22,26,30)}
    head=ImageFont.truetype(bold,32);subhead=ImageFont.truetype(bold,22)
    bg='#f3f6fa';ink='#172b46';muted='#62758c'
    base=Image.new('RGB',(1280,960),bg);d=ImageDraw.Draw(base)
    def label(x,y,text,size=17,color=ink):d.text((x,y),text,font=fonts[size],fill=color)
    d.text((32,22),'FR3 实验｜画面 × 储备池同步',font=head,fill=ink)
    label(32,73,('真实相机 + 直接记录状态 / 按时间戳配对' if esn_enabled else '无 ESN 对照 / 神经元与模型输出仅记录，不参与控制')+('  /  中止记录' if aborted else ''),19,muted)
    d.text((32,109),'相机实拍',font=subhead,fill=ink)
    d.text((720,109),'储备池状态 h(t) · 128 个单元',font=subhead,fill=ink)
    label(720,145,'固定色标 [-1, +1]；每格为一个状态分量',17,muted)
    cmap=colormaps['RdBu_r']
    gradient=np.linspace(1,-1,304)[:,None]
    colorbar=Image.fromarray((cmap((gradient+1)/2)[...,:3]*255).astype(np.uint8)).resize((14,304))
    base.paste(colorbar,(1215,185));label(1233,181,'+1',15);label(1233,326,'0',15);label(1233,472,'−1',15)
    for r in range(8):label(689,190+38*r,str(r*16),13,muted)
    label(720,500,'按编号排列，不表示神经元空间拓扑',17,muted)
    label(32,631,'录像与状态以最近时间戳关联；头尾无状态覆盖部分已裁剪。',17,muted)
    colors=['#293550','#d75f48','#138e85','#4479c7','#9b65b4','#c18b36','#70863d']
    boxes=[(54,713,312,868),(364,713,622,868),(674,713,932,868)]
    values=[force[:,None],dh[:,None],actions]
    limits=[(0,max(25,float(force.max())*1.1)),(0,max(.02,float(dh.max())*1.15)),(-1.05,1.05)]
    titles=['ESN 输入外力 |F| · N','神经元状态变化 · RMS Δh','原始策略输出 · a0…a6']
    plotted=[]
    for k,(box,ys,lim,title) in enumerate(zip(boxes,values,limits,titles)):
        x0,y0,x1,y1=box;d.text((x0-8,674),title,font=subhead,fill=ink)
        d.rectangle(box,fill='white',outline='#d5dfe9')
        def position(x,y):return (int(x0+x/duration*(x1-x0)),int(y1-(y-lim[0])/(lim[1]-lim[0])*(y1-y0)))
        points=[]
        for j in range(ys.shape[1]):
            xy=[position(x,y) for x,y in zip(t,ys[:,j])];points.append(xy)
            d.line(xy,fill='#dbe2ea',width=1)
        plotted.append(points)
        for tick in (0,10,20,30):
            if tick>duration:continue
            x,_=position(tick,0);d.line((x,y0,x,y1),fill='#e6ecf2');label(x-7,y1+5,str(tick),15,muted)
        label(x1-25,y1+5,f'{duration:.1f}s',13,muted)
        for y in [lim[0],lim[1]]:
            _,py=position(0,y);label(x0-40,py-9,f'{y:.2g}',13,muted)
        if k==0:
            _,py=position(0,12);d.line((x0,py,x1,py),fill='#d4b394',width=1)
            label(x1-72,py-21,'门控 12 N',13,muted)
    traj_box=(984,713,1242,868)
    d.text((976,674),'末端运动轨迹 · XY (mm)',font=subhead,fill=ink)
    d.rectangle(traj_box,fill='white',outline='#d5dfe9')
    traj_pts,traj_bounds,ref_traj_pts=project_xy_box(ee_xyz,traj_box,extra_xyz_m=ref_xyz)
    if ref_traj_pts is not None:draw_dashed_polyline(d,ref_traj_pts,fill='#8fa1b5',width=1,dash_px=6.0,gap_px=4.0)
    elif len(traj_pts)>1:d.line(traj_pts,fill='#dbe2ea',width=1)
    sx,sy=traj_pts[0];d.ellipse((sx-3,sy-3,sx+3,sy+3),outline='#62758c',width=1)
    label(984,873,f'X:[{traj_bounds[0]:.0f},{traj_bounds[1]:.0f}] Y:[{traj_bounds[2]:.0f},{traj_bounds[3]:.0f}]',13,muted)
    label(32,914,'本轮提前中止，未完成往返；画面为退出前真实记录，非完整演示。' if aborted else '门控表示控制器判据，并非独立碰撞真值；配对时间差不等于物理端到端延迟。',17,'#ac482d' if aborted else muted)
    label(32,938,'a0：减速；a1–a6：平移/转动让位原始输出（并非直接下发力矩）。',13,muted)
    ff=imageio_ffmpeg.get_ffmpeg_exe()
    cmd=[ff,'-v','error','-f','rawvideo','-pix_fmt','rgb24','-s','1280x960','-r',str(fps),'-i','-',
         '-an','-c:v','libx264','-threads','2','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'camera_esn_full.mp4')]
    encoder=subprocess.Popen(cmd,stdin=subprocess.PIPE)
    manifest=[];peak=int(np.argmax(force));snapshot_saved=False
    try:
        for j,stamp in enumerate(timeline):
            ii=int(np.searchsorted(ct,stamp));candidates=[i for i in [ii-1,ii] if first<=i<=last]
            ci=min(candidates,key=lambda i:abs(int(ct[i])-int(stamp)))
            pair=pairs[ci];pi=int(pair['nearest_policy_sample']);ok=valid[ci] and abs(int(ct[ci])-int(stamp))<25000000
            frame=base.copy();d=ImageDraw.Draw(frame)
            image=Image.fromarray(cv2.cvtColor(raw[ci],cv2.COLOR_YUV2RGB_YUY2));frame.paste(image,(32,145))
            label(32,115,f'                         t = {(int(ct[ci])-int(pt[0]))/1e9:6.2f} s',17,muted)
            delta=float(pair['policy_minus_camera_ms'])
            if ok:
                rgb=(cmap((h[pi].reshape(8,16)+1)/2)[...,:3]*255).astype(np.uint8)
                frame.paste(Image.fromarray(rgb).resize((480,304),Image.Resampling.NEAREST),(720,185))
                for gx in range(17):d.line((720+gx*30,185,720+gx*30,489),fill='#ffffff',width=1)
                for gy in range(9):d.line((720,185+gy*38,1200,185+gy*38),fill='#ffffff',width=1)
                state=('接触门控：启用' if contact[pi] else '接触门控：未启用') if esn_enabled else 'ESN 旁路：仅记录'
                d.rounded_rectangle((951,25,1248,67),radius=10,fill='#fce7dc' if contact[pi] else '#dff0ef')
                label(970,30,state,22,'#ac482d' if contact[pi] else '#16776f')
                label(720,538,f'输入外力  {force[pi]:5.2f} N',26)
                label(720,578,f'状态变化 RMS  {dh[pi]:.4f}',19,muted)
                label(720,620,f'配对差 Δt = {delta:+.2f} ms',19,muted)
                for k,box in enumerate(boxes):
                    x0,y0,x1,y1=box
                    for channel,pts in enumerate(plotted[k]):
                        if pi>0:d.line(pts[:pi+1],fill=(['#cb6340','#3b6eae'][k] if k<2 else colors[channel]),width=2)
                    x=x0+(int(ct[ci])-int(pt[0]))/1e9/duration*(x1-x0)
                    d.line((x,y0,x,y1),fill=ink,width=1)
                for idx_seg in range(pi):
                    seg_color='#cb6340' if (contact[idx_seg] or contact[idx_seg+1]) else '#138e85'
                    d.line((traj_pts[idx_seg],traj_pts[idx_seg+1]),fill=seg_color,width=2)
                cx_pt,cy_pt=traj_pts[pi]
                d.ellipse((cx_pt-4,cy_pt-4,cx_pt+4,cy_pt+4),fill='#ac482d' if contact[pi] else '#172b46',outline='white',width=1)
                ex_mm,ey_mm,ez_mm=ee_xyz[pi]*1000.0
                dev_txt=f'  偏移 {ref_dev_mm[pi]:.0f}mm' if ref_dev_mm is not None else ''
                label(990,718,f'({ex_mm:.0f},{ey_mm:.0f},{ez_mm:.0f}){dev_txt}',13,ink)
                if ref_cam_uv is not None:
                    ref_pts=[(32+int(round(ref_cam_uv[k,0])),145+int(round(ref_cam_uv[k,1]))) for k in range(len(ref_cam_uv)) if ref_cam_ok[k]]
                    draw_dashed_polyline(d,ref_pts,fill='#e8eef5',width=2,dash_px=8.0,gap_px=6.0)
                if cam_uv is not None:
                    for idx_seg in range(pi):
                        if cam_uv_ok[idx_seg] and cam_uv_ok[idx_seg+1]:
                            p0=(32+int(round(cam_uv[idx_seg,0])),145+int(round(cam_uv[idx_seg,1])))
                            p1=(32+int(round(cam_uv[idx_seg+1,0])),145+int(round(cam_uv[idx_seg+1,1])))
                            seg_c='#ff5733' if (contact[idx_seg] or contact[idx_seg+1]) else '#18b8a8'
                            d.line((p0,p1),fill=seg_c,width=3)
                    if cam_uv_ok[pi]:
                        ux,vy=32+int(round(cam_uv[pi,0])),145+int(round(cam_uv[pi,1]))
                        d.ellipse((ux-6,vy-6,ux+6,vy+6),fill='#ff3b1f' if contact[pi] else '#00d2be',outline='white',width=2)
                    label(40,602,'虚线:无障碍参考  青:正常  橙:接触让位',13,'#ffffff')
            else:
                label(720,250,'无有效同步状态，未插值补造',22,'#ac482d')
            encoder.stdin.write(frame.tobytes())
            manifest.append({'video_frame':j,'video_seconds':j/fps,'timeline_monotonic_ns':int(stamp),
               'camera_frame':ci,'policy_sample':pi,'valid':int(ok),'policy_minus_camera_ms':delta,
               'camera_minus_video_grid_ms':(int(ct[ci])-int(stamp))/1e6})
            if ok and pi==peak and not snapshot_saved:
                frame.save(out/'contact_peak.png');snapshot_saved=True
            if j%300==0:print(f'fusion render {j}/{len(timeline)}',flush=True)
        encoder.stdin.close();rc=encoder.wait()
        if rc:raise RuntimeError(f'Encoder exit {rc}')
    finally:
        if encoder.poll() is None:encoder.terminate();encoder.wait()
    with (out/'video_frame_mapping.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=manifest[0]);w.writeheader();w.writerows(manifest)
    # Include the largest force event, the approach and the recovery. Original playback speed.
    center=(int(pt[peak])-start)/1e9;clip_start=max(0,center-3);clip_end=min(len(timeline)/fps,center+5)
    subprocess.run([ff,'-v','error','-ss',str(clip_start),'-i',str(out/'camera_esn_full.mp4'),'-t',str(clip_end-clip_start),
        '-an','-c:v','libx264','-threads','2','-preset','fast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'contact_highlight.mp4')],check=True)
    summary={'session':str(root),'esn_applied':esn_enabled,'execution_return_code':execution['return_code'],'aborted_cycle':aborted,'frames':len(timeline),'fps':fps,'video_duration_s':len(timeline)/fps,
       'source_camera_frames':n,'source_policy_samples':len(pr),'directly_logged_states':True,
       'synthetic_data':False,'force_input_peak_N':float(force[peak]),'force_peak_elapsed_s':float(pr[peak]['elapsed_s']),
       'max_pair_delta_ms':max(abs(x['policy_minus_camera_ms']) for x in manifest if x['valid']),
       'invalid_output_frames':sum(not x['valid'] for x in manifest),
       'head_tail_camera_frames_trimmed':first+(n-1-last),'camera_sequence_gaps':alignment['camera_sequence_gaps'],
       'ee_trajectory_source':ee_source,
       'ref_trajectory_source':ref_source,
       'max_ref_deviation_mm':float(ref_dev_mm.max()) if ref_dev_mm is not None else None,
       'camera_overlay_calib':str(calib_path) if calib_path else None,
       'highlight_seconds_in_full_video':[clip_start,clip_end],
       'clock_alignment_is_not_physical_latency_calibration':True,
       'policy_csv_sha256':hashlib.sha256(policy.read_bytes()).hexdigest()}
    (out/'render_summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2),flush=True)

if __name__=='__main__':main()
