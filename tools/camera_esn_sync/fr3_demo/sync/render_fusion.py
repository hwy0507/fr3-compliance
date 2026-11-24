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

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--session',type=Path,required=True)
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
    colorbar=Image.fromarray((cmap(gradient)[...,:3]*255).astype(np.uint8)).resize((14,304))
    base.paste(colorbar,(1215,185));label(1233,181,'+1',15);label(1233,326,'0',15);label(1233,472,'−1',15)
    for r in range(8):label(689,190+38*r,str(r*16),13,muted)
    label(720,500,'按编号排列，不表示神经元空间拓扑',17,muted)
    label(32,631,'录像与状态以最近时间戳关联；头尾无状态覆盖部分已裁剪。',17,muted)
    colors=['#293550','#d75f48','#138e85','#4479c7','#9b65b4','#c18b36','#70863d']
    boxes=[(54,713,380,868),(465,713,791,868),(875,713,1227,868)]
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
       'highlight_seconds_in_full_video':[clip_start,clip_end],
       'clock_alignment_is_not_physical_latency_calibration':True,
       'policy_csv_sha256':hashlib.sha256(policy.read_bytes()).hexdigest()}
    (out/'render_summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2),flush=True)

if __name__=='__main__':main()
