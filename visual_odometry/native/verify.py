"""Small PC check using the actual C core and Python scheduler (stdlib only).

Run: python -B native/verify.py
Build products remain in native/_build. No OpenMV, serial port or firmware flash.
"""
import ctypes as T
import math
import os
from pathlib import Path
import subprocess
import sys
import types
import time

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.dont_write_bytecode = True
import config as C
import algorithm as A

F = T.c_float
FP = T.POINTER(F)
RNC = C.R_BC
WIDTH, HEIGHT = 320, 240


def inverse_rotate(r, v):
    x,y,z=v
    return (r[0]*x+r[3]*y+r[6]*z,r[1]*x+r[4]*y+r[7]*z,r[2]*x+r[5]*y+r[8]*z)


def undistort(u, v):
    """Independent PC geometry oracle; no unused helper is loaded on OpenMV."""
    xd,yd=(u-C.CX)/C.FX,(v-C.CY)/C.FY
    x,y=xd,yd
    k1,k2,p1,p2,k3=C.DIST_COEFFS
    for _ in range(6):
        r=x*x+y*y;k=1+k1*r+k2*r*r+k3*r*r*r
        x,y=(xd-2*p1*x*y-p2*(r+2*x*x))/k,(yd-p1*(r+2*y*y)-2*p2*x*y)/k
    return x,y


def build():
    folder = ROOT / 'native' / '_build'
    folder.mkdir(exist_ok=True)
    dll = folder / ('vo_core.dll' if os.name == 'nt' else 'vo_core.so')
    env = dict(os.environ, TMP=str(folder), TEMP=str(folder), TMPDIR=str(folder))
    subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic',
                    '-O2', '-shared', '-fPIC', '-DVO_TESTING', str(ROOT/'native'/'vo_core.c'),
                    str(ROOT/'native'/'vo_fast.c'),
                    '-o', str(dll), '-lm'], check=True, env=env)
    lib = T.CDLL(str(dll))
    lib.vo_context_bytes.restype = T.c_size_t
    lib.vo_init.argtypes = [T.c_void_p, FP]
    lib.vo_reset.argtypes = [T.c_void_p]
    lib.vo_step.argtypes = [T.c_void_p, T.c_void_p, T.c_size_t, FP, T.c_int64, FP, F]
    lib.vo_test_pairs.argtypes = [T.c_void_p, FP, FP, FP, T.c_int]
    lib.vo_test_limit.argtypes = [T.c_void_p, T.c_int, T.c_int]
    lib.vo_test_spoil_key.argtypes = [T.c_void_p]
    lib.vo_test_search.argtypes = [T.c_void_p,T.c_void_p,T.c_void_p,F,F,F,F,FP]
    lib.vo_test_search.restype = T.c_int
    lib.vo_profile.argtypes = [T.c_void_p, T.c_void_p]
    lib.vo_get_timings.argtypes = [T.c_void_p]
    lib.vo_get_timings.restype = T.POINTER(T.c_uint32)
    lib.vo_get_result.argtypes = [T.c_void_p]
    lib.vo_get_result.restype = FP
    return lib


def floats(value):
    return (F*len(value))(*value)


class HostCore:
    """Buffer-compatible host adapter; all tracking/geometry still executes C."""
    def __init__(self, settings):
        self.ctx = T.create_string_buffer(LIB.vo_context_bytes())
        assert LIB.vo_init(self.ctx, floats(settings)) == 1

    def reset(self):
        LIB.vo_reset(self.ctx)

    def output(self, out):
        values = LIB.vo_get_result(self.ctx)
        for i in range(A.RESULT_FLOATS):
            out[i] = values[i]

    def step(self, image, meta, stamp, prediction, out):
        if not isinstance(stamp, int): stamp=stamp[0]+(stamp[1]<<32)
        data = (T.c_uint8*len(image)).from_buffer(image)
        LIB.vo_step(self.ctx, data, 320, floats(meta), stamp, floats(prediction), prediction[2])
        self.output(out)

    def profile(self, enabled):
        self.callback = T.CFUNCTYPE(T.c_uint32)(lambda: (time.monotonic_ns()//1000)&0xffffffff) if enabled else None
        LIB.vo_profile(self.ctx, self.callback)

    def timings(self, out):
        values = LIB.vo_get_timings(self.ctx)
        for i in range(6): out[i] = values[i]

    def limit(self, count, topup=0):
        LIB.vo_test_limit(self.ctx, count, topup)


class Image(bytearray):
    def width(self): return WIDTH
    def height(self): return HEIGHT
    def format(self): return 1
    def find_keypoints(self, **kwargs): raise AssertionError('ORB must not run')


def q_yaw(angle):
    return (math.cos(angle/2), 0, 0, math.sin(angle/2))


def hash_value(x, y):
    n = (x*73856093 ^ y*19349663) & 0xffffffff
    n = ((n ^ (n >> 13))*1274126177) & 0xffffffff
    return ((n ^ (n >> 16)) & 255)


def noise(x, y, period):
    x, y = x/period, y/period
    ix, iy = math.floor(x), math.floor(y)
    ax, ay = x-ix, y-iy
    return ((1-ay)*((1-ax)*hash_value(ix,iy)+ax*hash_value(ix+1,iy))
            +ay*((1-ax)*hash_value(ix,iy+1)+ax*hash_value(ix+1,iy+1)))


def render(position=(0, 0), q=(1, 0, 0, 0), height=1.5, arm=(0, 0, 0)):
    rotation = A.matmul(A.q_matrix(q), C.R_BC)
    lever = A.rotate(A.q_matrix(q), arm)
    im = Image(WIDTH*HEIGHT)
    for i, (x, y) in enumerate(RAYS):
        ray = A.rotate(rotation, (x, y, 1))
        if ray[2] <= C.MIN_RAY_DOWN:
            continue
        n = position[0]+lever[0]+height*ray[0]/ray[2]
        e = position[1]+lever[1]+height*ray[1]/ray[2]
        u, v = 180+e*200, 150-n*200
        im[i] = max(0, min(255, round(.7*noise(u,v,3)+.3*noise(u,v,8))))
    return im


def pixel_of(ground, position, q, height, arm=(0, 0, 0)):
    r = A.q_matrix(q)
    lever = A.rotate(r, arm)
    ray = inverse_rotate(A.matmul(r,C.R_BC),
        (ground[0]-position[0]-lever[0],ground[1]-position[1]-lever[1],height))
    x,y = ray[0]/ray[2],ray[1]/ray[2]
    k1,k2,p1,p2,k3 = C.DIST_COEFFS
    r2 = x*x+y*y
    radial = 1+k1*r2+k2*r2*r2+k3*r2*r2*r2
    return (C.FX*(x*radial+2*p1*x*y+p2*(r2+2*x*x))+C.CX,
            C.FY*(y*radial+p1*(r2+2*y*y)+2*p2*x*y)+C.CY)


def check_position(motion, expected, tolerance=.012):
    assert motion['status']==2, motion
    assert math.dist(motion['position'],expected)<tolerance, (motion,expected)


def check_frontend():
    engine = A.FeatureEngine()
    first = render()
    assert engine.process(first,(1,0,0,0),1.5,(0,0,0),100000)['status']==1
    # The camera buffer may be immediately reused; owned references must survive.
    first[:] = bytes(len(first))
    slow = engine.process(render((.012,-.009)),(1,0,0,0),1.5,(0,0,0),133333)
    check_position(slow,(.012,-.009),.006)
    # Fixed anchor subpixel repeat, then bounded larger displacement with prediction.
    key_ids = []
    for k in range(2,8):
        p = (.012*k,-.009*k)
        motion = engine.process(render(p),(1,0,0,0),1.5,(0,0,0),100000+33333*k)
        check_position(motion,p)
        key_ids.append(motion['keyframe'])
    assert len(set(key_ids))==1, key_ids
    last_key = engine.raw[12]
    for k in range(8,33):
        p = (.012*k,-.009*k)
        motion = engine.process(render(p),(1,0,0,0),1.5,(0,0,0),100000+33333*k)
        if motion['status']!=2:
            print('Failed frame:', k, p, 'prediction:', list(engine.prediction), engine.diagnostics)
        check_position(motion,p,.02)
    assert motion['keyframe']>last_key, motion
    # A prediction into a blank image cannot become a valid visual measurement.
    lost = engine.process(Image(WIDTH*HEIGHT),(1,0,0,0),1.5,(0,0,0),1200000)
    assert lost['status']==0, lost
    restored = engine.process(render(p),(1,0,0,0),1.5,(0,0,0),1233333)
    check_position(restored,p,.02)
    print('PASS: subpixel tracking, N/E scale, buffer ownership, keyframe continuity, blank/recovery')


def check_rotations():
    for q,height,arm in [(q_yaw(.12),1.5,(0,0,0)),
                         ((math.cos(.08),math.sin(.08),0,0),1.65,(.08,-.04,.03))]:
        engine = A.FeatureEngine()
        engine.process(render(arm=arm),(1,0,0,0),1.5,arm,100000)
        current = engine.process(render(q=q,height=height,arm=arm),q,height,arm,133333)
        check_position(current,(0,0),.015)
    print('PASS: attitude compensation, height change and camera lever arm')


def check_geometry():
    core = HostCore(A.native_settings())
    true_q = q_yaw(.045)
    position = (.09,-.05)
    rm = C.R_BC+(0,0,0,1.5)
    current = A.matmul(A.q_matrix(true_q), C.R_BC)+(0,0,0,1.5)
    pairs=[]
    for v in (40,80,120,160,200):
        for u in (40,90,140,190,240,280):
            x,y = undistort(u,v)
            g = A.rotate(C.R_BC,(x,y,1))
            uv = pixel_of((1.5*g[0]/g[2],1.5*g[1]/g[2]),position,true_q,1.5)
            pairs.extend((u,v,*uv))
    for i in range(0,20,4):
        pairs[4*i+2],pairs[4*i+3] = 45+i, 210-i
    LIB.vo_test_pairs(core.ctx,floats(rm),floats(current),floats(pairs),len(pairs)//4)
    raw=[0]*A.RESULT_FLOATS;core.output(raw)
    assert raw[0]==2 and math.dist(raw[3:5],position)<.003, raw
    assert raw[6]<raw[5], raw
    # Two collinear features are sufficient for translation, not for full pose.
    clean=[]
    for u,v in ((70,90),(240,90)):
        x,y=undistort(u,v);g=A.rotate(C.R_BC,(x,y,1))
        clean.extend((u,v,*pixel_of((1.5*g[0],1.5*g[1]),position,true_q,1.5)))
    LIB.vo_test_pairs(core.ctx,floats(rm),floats(current),floats(clean),2)
    core.output(raw)
    assert raw[0]==2 and raw[16]==1 and math.dist(raw[3:5],position)<.003,raw
    clean[6]+=20
    LIB.vo_test_pairs(core.ctx,floats(rm),floats(current),floats(clean),2)
    core.output(raw)
    assert raw[0]==0,raw
    print('PASS: compensated translation, outlier rejection, collinear two-point solve and conflicting pair rejection')


def check_sparse_images():
    saved=C.MAX_KEYPOINTS
    try:
        for count in (2,3):
            C.MAX_KEYPOINTS=count
            engine=A.FeatureEngine()
            assert engine.process(render(),(1,0,0,0),1.5,(0,0,0),100000)['status']==1
            for k in range(1,5):
                p=(.003*k,-.002*k)
                motion=engine.process(render(p),(1,0,0,0),1.5,(0,0,0),100000+k*33333)
                check_position(motion,p,.008)
                assert motion['sparse'] and motion['inliers']==count,motion
                assert motion['attempts']==1 and motion['detect_cells']<=6 and motion['added']<=4,motion
            # KLT and sparse uniqueness both tolerate a uniform brightness offset.
            bright=render((.015,-.01))
            bright[:]=bytes(min(255,value+20) for value in bright)
            check_position(engine.process(bright,(1,0,0,0),1.5,(0,0,0),266665),(.015,-.01),.008)
        C.MAX_KEYPOINTS=saved
        engine=A.FeatureEngine()
        engine.process(render(),(1,0,0,0),1.5,(0,0,0),100000)
        engine.core.limit(2)
        motion=engine.process(render((.003,-.002)),(1,0,0,0),1.5,(0,0,0),133333)
        check_position(motion,(.003,-.002),.008)
        assert motion['inliers']==2 and motion['sparse'],motion
        engine.core.limit(1)
        invalid=engine.process(render((.006,-.004)),(1,0,0,0),1.5,(0,0,0),166666)
        assert invalid['status']==0 and invalid['attempts']==1,invalid
        # Perfectly repeating texture must not give an overconfident sparse fix.
        periodic=Image(hash_value(x%6,y%6) for y in range(HEIGHT) for x in range(WIDTH))
        engine=A.FeatureEngine()
        assert engine.process(periodic,(1,0,0,0),1.5,(0,0,0),100000)['status']==1
        engine.core.limit(2)
        invalid=engine.process(periodic,(1,0,0,0),1.5,(0,0,0),133333)
        assert invalid['status']==0 and engine.diagnostics['vision_reason']=='AMBIGUOUS_MATCH',invalid
    finally:
        C.MAX_KEYPOINTS=saved
    print('PASS: actual images with 2/3 points, rich-to-sparse continuity, single-point invalidity and repetitive-texture rejection')


def check_fast_equivalence():
    import random
    rng=random.Random(4519)
    LIB.vo_fast_test.argtypes=[T.POINTER(T.c_uint8),T.POINTER(T.c_int),T.c_int]
    dx=(0,1,2,3,3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1)
    dy=(3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1,0,1,2,3)
    offsets=(T.c_int*16)(*(y*9+x for x,y in zip(dx,dy)))
    for case in range(300):
        data=(T.c_uint8*81)(*(rng.randrange(256) for _ in range(81)))
        point=T.cast(T.byref(data,40),T.POINTER(T.c_uint8))
        diff=[int(data[40+o])-data[40] for o in offsets]
        for threshold in (5,15,60,120):
            expected=any(all(diff[(i+k)%16]>threshold for k in range(9)) or
                         all(diff[(i+k)%16]<-threshold for k in range(9)) for i in range(16))
            assert bool(LIB.vo_fast_test(point,offsets,threshold))==expected,(case,threshold)
    print('PASS: OpenMV generated FAST-9 equals mathematical 9/16 criterion on 1200 random cases')


def check_controller():
    from comm import History
    from control import VOController
    from output import Output
    from main import validate_config
    validate_config()
    h=History(3)
    for i in range(5): h.append((i,))
    assert [x[0] for x in h]==[2,3,4] and [x[0] for x in reversed(h)]==[4,3,2]
    class Clock:
        now=100000
        def now_us(self): return self.now
    class Link:
        boot_generation=time_generation=0
        range_ambiguous=False
        vibration=None
        attitudes=History()
        ranges=History()
        clock=Clock()
        sent=[]
        def connected(self,now=None): return True
        def synced(self,now=None): return True
        def fc_time(self,stamp): return stamp
        def send_odometry(self,result): self.sent.append(result['timestamp_us'])
        def cancel_odometry(self): pass
    link=Link()
    controller=VOController()
    output=Output(link,display=False)
    for i in range(8):
        stamp=100000+i*33333
        link.clock.now=stamp
        link.attitudes.append((stamp,(1,0,0,0),(0,0,0),stamp))
        link.ranges.append((stamp,1.5,.05,8,100,None,stamp))
        result=controller.process(render(),stamp,link)
        output.update(result,0)
    assert result['valid'] and result['approximate'],result
    assert not link.sent
    saved = C.SEND_ODOMETRY, C.SENSOR_TIMING_CONFIRMED
    C.SEND_ODOMETRY=C.SENSOR_TIMING_CONFIRMED=True
    publication=dict(result,body_valid=True,approximate=False,quality=80)
    output.update(publication,0)
    output.update(publication,0)
    assert link.sent==[result['timestamp_us']]
    C.SEND_ODOMETRY,C.SENSOR_TIMING_CONFIRMED=saved
    old=C.LOST_RESET_US,C.TRACK_HOLD_US
    C.LOST_RESET_US,C.TRACK_HOLD_US=50000,20000
    for i in range(3):
        stamp+=33333;link.clock.now=stamp
        link.attitudes.append((stamp,(1,0,0,0),(0,0,0),stamp))
        link.ranges.append((stamp,1.5,.05,8,100,None,stamp))
        result=controller.process(Image(WIDTH*HEIGHT),stamp,link)
        assert not result['valid']
    C.LOST_RESET_US,C.TRACK_HOLD_US=old
    assert result['reset_counter']==1,result
    print('PASS: native/Python controller, histories, display, publish-once and explicit reset')


def check_sparse_controller():
    from comm import History
    from control import VOController
    saved={name:getattr(C,name) for name in ('CAMERA_POS_B_M','TOF_POS_B_M','CAMERA_ORIENTATION_CONFIRMED')}
    class Engine:
        count=12
        diagnostics={'vision_reason':'OK','sparse':False}
        def reset(self): pass
        def accept_variance(self, motion, variance): self.variance=variance
        def process(self,*args):
            n=self.count
            return dict(status=2 if n>=2 else 0,position=(0.1,0.2),delta=(0.0,0.0),
                inliers=n,matches=n,keyframe=0,reference_height=1.5,
                sample_variance=(0.0,0.0),base_variance=(0.0,0.0),ratio=1.0,
                effective_count=float(n),cells=min(6,n),spread=.2,residual_px=0.0)
    class Clock:
        now=100000
        def now_us(self): return self.now
    class Link:
        boot_generation=time_generation=0
        range_ambiguous=False
        vibration=None
        def __init__(self): self.clock=Clock();self.attitudes=History();self.ranges=History()
        def connected(self,now=None): return True
        def synced(self,now=None): return True
        def fc_time(self,stamp): return stamp
    try:
        C.CAMERA_POS_B_M=C.TOF_POS_B_M=(0,0,0);C.CAMERA_ORIENTATION_CONFIRMED=True
        engine=Engine();controller=VOController(engine);link=Link()
        def frame(count):
            engine.count=count;stamp=link.clock.now+33333;link.clock.now=stamp
            link.attitudes.append((stamp,(1,0,0,0),(0,0,0),stamp))
            link.ranges.append((stamp,1.5,.05,8,100,None,stamp))
            return controller.process(Image(),stamp,link)
        for _ in range(8): normal=frame(12)
        assert normal['valid'],normal
        triple=frame(3);dual=frame(2);restored=frame(12)
        assert all(x['valid'] for x in (triple,dual,restored)),(triple,dual,restored)
        assert triple['reason']==dual['reason']=='SPARSE_TRACKING'
        assert dual['variance'][0]>triple['variance'][0]>normal['variance'][0]>0
        assert dual['quality']<normal['quality'] and restored['position']==normal['position']
        assert controller.reset_counter==0
        assert frame(1)['predicted']
        assert frame(2)['valid']  # short failure resumes on the next real fix
    finally:
        for name,value in saved.items(): setattr(C,name,value)
    print('PASS: unified sparse confidence, increasing variance and immediate recovery from a short failure')


def check_brightness_search():
    core=HostCore(A.native_settings())
    reference=Image(90+hash_value(x,y)%61 for y in range(HEIGHT) for x in range(WIDTH))
    ref=(T.c_uint8*len(reference)).from_buffer(reference)
    for dx,dy,offset in ((6,-3,40),(-5,4,-40),(0,0,50)):
        current=Image(WIDTH*HEIGHT)
        for y in range(HEIGHT):
            for x in range(WIDTH):
                if 0<=x-dx<WIDTH and 0<=y-dy<HEIGHT:
                    current[y*WIDTH+x]=reference[(y-dy)*WIDTH+x-dx]+offset
        im=(T.c_uint8*len(current)).from_buffer(current)
        out=(F*2)()
        assert LIB.vo_test_search(core.ctx,ref,im,160,120,160,120,out)==1
        assert tuple(out)==(160+dx,120+dy),(dx,dy,offset,tuple(out))
    print('PASS: bounded fallback search tolerates +/-brightness offsets and finds known integer translations')


def check_confidence_prediction():
    from control import confidence
    from main import validate_config
    good=lambda variance,n=16,cells=6,ratio=1,weight=1: confidence((variance,variance),n,cells,ratio,n*weight)
    scores=[good(v) for v in (0,.001,.01,.1,1)]
    assert all(a>b for a,b in zip(scores,scores[1:])),scores
    assert good(.001,16)==good(.001,24)==good(.001,64)
    assert good(.001,2)<good(.001,16)
    assert good(.001)-good(.001,cells=0)<=10
    assert good(.001)-good(.001,ratio=0)<=10
    assert good(.001)-good(.001,cells=0,ratio=0)<=20
    assert confidence((float('nan'),0),16,6,1,16)==0
    assert confidence((0,0),1,6,1,1)==0
    saved=C.CONFIDENCE_SIGMA0_M
    try:
        before=good(.01);C.CONFIDENCE_SIGMA0_M*=2
        assert good(.01)>before
        for name,value in (('CONFIDENCE_SIGMA0_M',0),('TRACK_HOLD_US',C.LOST_RESET_US),
                           ('PREDICTION_HISTORY_SIZE',1)):
            old=getattr(C,name);setattr(C,name,value)
            try:
                try: validate_config()
                except ValueError: pass
                else: raise AssertionError('invalid parameter accepted: '+name)
            finally: setattr(C,name,old)
    finally:
        C.CONFIDENCE_SIGMA0_M=saved
    history=A.MotionHistory();origin=1<<40
    for dt in (0,25000,58000,100000,145000):
        history.add(origin+dt,(.4*dt*1e-6,-.2*dt*1e-6))
    last_stamp=history.stamp
    n=len(history.samples)
    for dt in (100000,500000,C.TRACK_HOLD_US):
        position,extra=history.project(last_stamp+dt)
        expected=(.4*(145000+dt)*1e-6,-.2*(145000+dt)*1e-6)
        assert math.dist(position,expected)<1e-10,(position,expected)
        assert min(extra)>0 and history.stamp==last_stamp and len(history.samples)==n
    assert history.project(last_stamp+C.TRACK_HOLD_US+1) is None
    history.reset();history.add(origin,(1,2))
    assert history.project(origin+100000)[0]==(1,2)
    print('PASS: single confidence weighting, 16-point saturation, adjustable scales and bounded uint64-time motion prediction')


def check_prediction_lifecycle():
    from comm import History
    from control import VOController
    from output import publication_block_mask,PublicationGate
    from test import TestSession
    saved={name:getattr(C,name) for name in ('CAMERA_POS_B_M','TOF_POS_B_M',
        'CAMERA_ORIENTATION_CONFIRMED','SENSOR_TIMING_CONFIRMED','TRACK_HOLD_US','LOST_RESET_US')}
    class Engine:
        success=True
        diagnostics={'vision_reason':'MATCH_COUNT'}
        def reset(self): pass
        def accept_variance(self,*args): pass
        def process(self,image,q,height,arm,stamp,rotation=None):
            t=stamp*1e-6;n=24 if self.success else 0
            return dict(status=2 if self.success else 0,position=(.4*t,-.2*t),delta=(.02,-.01),
                inliers=n,matches=n,keyframe=0,reference_height=1.5,sample_variance=(0,0),
                base_variance=(0,0),ratio=1,cells=6,effective_count=n,residual_px=0)
    class Clock:
        now=100000
        def now_us(self): return self.now
    class Link:
        boot_generation=time_generation=0
        range_ambiguous=False
        vibration=None
        def __init__(self): self.clock=Clock();self.attitudes=History();self.ranges=History()
        def connected(self,now=None): return True
        def synced(self,now=None): return True
        def fc_time(self,stamp): return stamp
    try:
        C.CAMERA_POS_B_M=C.TOF_POS_B_M=(0,0,0)
        C.CAMERA_ORIENTATION_CONFIRMED=C.SENSOR_TIMING_CONFIRMED=True
        engine=Engine();link=Link();controller=VOController(engine)
        def frame(stamp,success=True):
            engine.success=success;link.clock.now=stamp
            link.attitudes.append((stamp,(1,0,0,0),(0,0,0),stamp))
            link.ranges.append((stamp,1.5,.05,8,100,None,stamp))
            return controller.process(Image(),stamp,link)
        for i in range(8): measured=frame(100000+i*50000)
        assert measured['valid'] and measured['measured']
        anchor=measured['timestamp_us']
        before_count=len(controller.predictor.samples)
        first=frame(anchor+50000,False)
        second=frame(anchor+100000,False)
        assert all(r['valid'] and r['predicted'] and not r['measured'] for r in (first,second))
        for r in (first,second):
            assert r['measurement_timestamp_us']==anchor
            assert math.dist(r['position'],(.4*r['timestamp_us']*1e-6,-.2*r['timestamp_us']*1e-6))<1e-10
            assert publication_block_mask(r,link,r['timestamp_us']) & 128
            assert not PublicationGate().accept(r,r['timestamp_us'],0)
        assert controller.last_success_stamp==anchor and len(controller.predictor.samples)==before_count
        assert second['variance'][0]>first['variance'][0]>measured['variance'][0]
        assert second['quality']<=first['quality']<=measured['quality']
        recovered=frame(anchor+150000)
        assert recovered['valid'] and recovered['measured'] and not recovered['predicted']
        # Prediction frames remain new input images, but do not count as fixes.
        session=TestSession(1);session.link=link;session.start=0
        for r in (measured,first,second,recovered): session.observe(r,r['timestamp_us'])
        assert (session.frames,session.valid_frames,session.predicted_frames)==(4,2,2)
        assert session.lost_episodes==0 and session.quality_count==2
        assert session.state(second,second['timestamp_us']) & 7 == 4
        anchor=recovered['timestamp_us']
        last_prediction=frame(anchor+C.TRACK_HOLD_US,False)
        assert last_prediction['valid'] and last_prediction['predicted']
        lost=frame(anchor+C.TRACK_HOLD_US+1,False)
        assert not lost['valid'] and lost['reason']=='LOST' and lost['reset_counter']==0
        reconfirm=frame(anchor+C.TRACK_HOLD_US+50000)
        assert not reconfirm['valid'] and reconfirm['reason']=='POSE_RECOVERING'
        frame(anchor+C.TRACK_HOLD_US+100000)
        recovered=frame(anchor+C.TRACK_HOLD_US+150000)
        assert recovered['valid'] and recovered['reset_counter']==0
        anchor=recovered['timestamp_us']
        assert frame(anchor+C.LOST_RESET_US-1,False)['reset_counter']==0
        reset=frame(anchor+C.LOST_RESET_US,False)
        assert reset['reason']=='REINITIALIZING' and reset['reset_counter']==1
        assert not reset['valid'] and controller.predictor.stamp is None
        # A hard input generation change cannot reuse the previous trajectory.
        for i in range(8): frame(anchor+C.LOST_RESET_US+50000*(i+1))
        assert controller.tracking_confirmed
        link.boot_generation+=1
        changed=frame(link.clock.now+50000,False)
        assert not changed['predicted'] and not changed['valid']
        # The hold and reset limits really follow config.py, including the boundary.
        C.TRACK_HOLD_US=40000;C.LOST_RESET_US=120000
        engine=Engine();link=Link();controller=VOController(engine)
        for i in range(8): measured=frame(100000+i*20000)
        assert measured['valid']
        anchor=measured['timestamp_us']
        assert frame(anchor+40000,False)['predicted']
        assert not frame(anchor+40001,False)['valid']
        assert frame(anchor+120000,False)['reason']=='REINITIALIZING'
    finally:
        for name,value in saved.items(): setattr(C,name,value)
    print('PASS: predicted positions, no feedback/publication/stat inflation, immediate recovery, configurable 1s/3s boundaries and reset epochs')


def check_work_bounds():
    engine=A.FeatureEngine()
    engine.process(render(),(1,0,0,0),1.5,(0,0,0),100000)
    engine.core.limit(2,4)
    motion=engine.process(render((.002,-.001)),(1,0,0,0),1.5,(0,0,0),133333)
    assert motion['status']==2 and motion['detect_cells']<=6 and motion['added']<=4,motion
    # Configured search radius is a residual budget, not permission to fabricate a fix.
    invalid=engine.process(render((.4,.25)),(1,0,0,0),1.5,(0,0,0),166666)
    assert invalid['status']==0 and invalid['attempts']==1,invalid
    assert invalid['sad_candidates']<=64*(17*17)+3*(13*13),invalid
    empty=A.FeatureEngine()
    empty.process(Image(WIDTH*HEIGHT),(1,0,0,0),1.5,(0,0,0),100000)
    motion=empty.process(Image(WIDTH*HEIGHT),(1,0,0,0),1.5,(0,0,0),133333)
    assert motion['status']==-1 and motion['detect_cells']<=6,motion
    # Mode2 uses actual C clocks; the production path has no clock callback.
    class Probe:
        samples=[]
        def native(self, values): self.samples.append(tuple(values))
    engine=A.FeatureEngine(Probe())
    engine.process(render(),(1,0,0,0),1.5,(0,0,0),100000)
    engine.process(render((.002,-.001)),(1,0,0,0),1.5,(0,0,0),133333)
    assert any(sum(x)>0 for x in engine.probe.samples)
    assert engine.profile_enabled
    engine.probe=None
    engine.process(render((.004,-.002)),(1,0,0,0),1.5,(0,0,0),166666)
    assert not engine.profile_enabled and not any(LIB.vo_get_timings(engine.core.ctx)[i] for i in range(6))
    engine=A.FeatureEngine();image=render()
    engine.process(image,(1,0,0,0),1.5,(0,0,0),100000)
    LIB.vo_test_spoil_key(engine.core.ctx)
    failures=0
    for i in range(1,17):
        motion=engine.process(image,(1,0,0,0),1.5,(0,0,0),100000+i*33333)
        failures+=motion['status']!=2
    assert failures==1 and motion['keyframe']>=1,(failures,motion)
    print('PASS: stale keyframe replaced only after a real neighbour-frame fix, no periodic anchor failures')
    print('PASS: refill/search limits, no large-motion false fix, bounded empty initialization and opt-in native timing')


def check_micropython(executable):
    folder = ROOT/'native'/'_build'
    (folder/'reference.gray').write_bytes(render())
    (folder/'shifted.gray').write_bytes(render((.012,-.009)))
    script = r'''
import sys
sys.path.insert(0, %r)
from array import array
import rm_vo, algorithm as A, config as C
assert rm_vo.ABI_VERSION==3 and rm_vo.RESULT_FLOATS==23
core=rm_vo.Core(A.native_settings())
core.profile(True)
meta=array('f', C.R_BC+(0,0,0,1.5))
stamp=array('I',(100000,1))
prediction=array('f',(0,0,8))
out=array('f',[0]*23)
with open(%r,'rb') as f: reference=bytearray(f.read())
with open(%r,'rb') as f: shifted=bytearray(f.read())
core.step(reference,meta,stamp,prediction,out)
assert out[0]==1, out
reference[:]=bytes(len(reference))
stamp[0]=133333
core.step(shifted,meta,stamp,prediction,out)
assert out[0]==2 and abs(out[1]-.012)<.006 and abs(out[2]+.009)<.006,out
assert out[19]==1 and len(out)==23,out
stamp[0]=166666;stamp[1]=0
core.step(shifted,meta,stamp,prediction,out)
assert out[0]==-2,out
stamp[0]=0;stamp[1]=2
core.step(shifted,meta,stamp,prediction,out)
assert out[0]==2,out
timing=array('I',[0]*6)
core.timings(timing)
assert sum(timing)>0,timing
try:
    core.step(b'x',meta,stamp,prediction,out)
    raise AssertionError('short buffer accepted')
except ValueError:
    pass
core.reset()
stamp[0]=166666
core.step(shifted,meta,stamp,prediction,out)
assert out[0]==1
print('PASS: actual MicroPython ABI3 binding, uint64 time, buffers, native timings and reset')
class Image:
    def __init__(self,data): self.data=data
    def width(self): return 320
    def height(self): return 240
    def format(self): return 1
    def find_keypoints(self,**kwargs): raise AssertionError('ORB must not run')
class ImageModule:
    GRAYSCALE=1
sys.modules['image']=ImageModule()
class BorrowedBuffer:
    # OpenMV supplies a native Image buffer; the Windows-only image facade
    # unwraps its bytearray while all numeric/buffer work still runs real C.
    def __init__(self,core): self.core=core
    def reset(self): self.core.reset()
    def profile(self,enabled): self.core.profile(enabled)
    def timings(self,out): self.core.timings(out)
    def step(self,image,*args): self.core.step(image.data,*args)
class Probe:
    count=0
    def native(self,values): self.count+=1
C.MAX_KEYPOINTS=2
probe=Probe();engine=A.FeatureEngine(probe);engine.core=BorrowedBuffer(engine.core)
with open(%r,'rb') as f: im=Image(bytearray(f.read()))
assert engine.process(im,(1,0,0,0),1.5,(0,0,0),(1<<32)+100000)['status']==1
motion=engine.process(Image(shifted),(1,0,0,0),1.5,(0,0,0),(1<<32)+133333)
assert motion['status']==2 and motion['sparse'] and motion['attempts']==1,motion
assert abs(motion['position'][0]-.012)<.006 and probe.count==2,motion
print('PASS: real MicroPython frontend, two-point native flow and profiling, no ORB')
history=A.MotionHistory()
for i in range(5): history.add((1<<40)+i*30000,(.01*i,-.02*i))
position,extra=history.project((1<<40)+220000)
assert abs(position[0]-(.04+.01/.03*.1))<1e-6 and min(extra)>0,position
assert len(history.samples)==5 and history.stamp==(1<<40)+120000
assert history.project(history.stamp+C.TRACK_HOLD_US+1) is None
from control import confidence
assert confidence((.001,.001),16,6,1,16)==confidence((.001,.001),24,6,1,24)
assert confidence((.001,.001),16,6,1,16)>confidence((.1,.1),16,6,1,16)
print('PASS: actual MicroPython temporal regression, prediction provenance and unified confidence')
''' % (str(ROOT),str(folder/'reference.gray'),str(folder/'shifted.gray'),str(folder/'reference.gray'))
    subprocess.run([str(executable),'-X','heapsize=4M','-c',script],cwd=ROOT,check=True)


if __name__=='__main__':
    LIB=build()
    sys.modules['rm_vo']=types.SimpleNamespace(Core=HostCore,ABI_VERSION=3)
    sys.modules['image']=types.SimpleNamespace(GRAYSCALE=1,CORNER_FAST=0,CORNER_AGAST=1)
    for p in ROOT.glob('*.py'):
        compile(p.read_text(encoding='utf-8'),str(p),'exec')
    RAYS=[undistort(u,v) for v in range(HEIGHT) for u in range(WIDTH)]
    print('C context bytes:',LIB.vo_context_bytes(),flush=True)
    check_fast_equivalence()
    check_frontend()
    check_rotations()
    check_geometry()
    check_sparse_images()
    check_brightness_search()
    check_controller()
    check_sparse_controller()
    check_confidence_prediction()
    check_prediction_lifecycle()
    check_work_bounds()
    if '--micropython' in sys.argv:
        check_micropython(sys.argv[sys.argv.index('--micropython')+1])
    print('All PC checks passed. Hardware FPS/firmware build remain separate.')
