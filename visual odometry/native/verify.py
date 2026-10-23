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

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.dont_write_bytecode = True
import config as C
import algorithm as A

F = T.c_float
FP = T.POINTER(F)
RNC = C.R_BC
WIDTH, HEIGHT = 320, 240


def build():
    folder = ROOT / 'native' / '_build'
    folder.mkdir(exist_ok=True)
    dll = folder / ('vo_core.dll' if os.name == 'nt' else 'vo_core.so')
    env = dict(os.environ, TMP=str(folder), TEMP=str(folder), TMPDIR=str(folder))
    subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic',
                    '-O2', '-shared', '-fPIC', str(ROOT/'native'/'vo_core.c'),
                    '-o', str(dll), '-lm'], check=True, env=env)
    lib = T.CDLL(str(dll))
    lib.vo_context_bytes.restype = T.c_size_t
    lib.vo_init.argtypes = [T.c_void_p, FP]
    lib.vo_reset.argtypes = [T.c_void_p]
    lib.vo_step.argtypes = [T.c_void_p, T.c_void_p, T.c_size_t, FP, T.c_int64, FP, F]
    lib.vo_recover.argtypes = [T.c_void_p, T.c_void_p, T.c_size_t, FP, T.c_int64, FP, T.c_size_t]
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
        for i in range(23):
            out[i] = values[i]

    def step(self, image, meta, stamp, prediction, out):
        if not isinstance(stamp, int): stamp=stamp[0]+(stamp[1]<<32)
        data = (T.c_uint8*len(image)).from_buffer(image)
        LIB.vo_step(self.ctx, data, 320, floats(meta), stamp, floats(prediction), prediction[2])
        self.output(out)

    def recover(self, image, meta, stamp, pairs, out):
        if not isinstance(stamp, int): stamp=stamp[0]+(stamp[1]<<32)
        data = (T.c_uint8*len(image)).from_buffer(image)
        LIB.vo_recover(self.ctx, data, 320, floats(meta), stamp, floats(pairs), len(pairs)//4)
        self.output(out)


class Image(bytearray):
    def width(self): return WIDTH
    def height(self): return HEIGHT
    def format(self): return 1
    def find_keypoints(self, **kwargs): return None


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
    ray = A.inverse_rotate(A.matmul(r,C.R_BC),
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
    last_key = engine.raw[14]
    for k in (8,10,12,14,16,18,20,22,24,26,28,30,32):
        p = (.014*k,-.004*k)
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
    engine = A.FeatureEngine()
    engine.process(render(),(1,0,0,0),1.5,(0,0,0),100000)
    true_q = q_yaw(.045)
    position = (.09,-.05)
    pairs=[]
    for v in (40,80,120,160,200):
        for u in (40,90,140,190,240,280):
            x,y = A.undistort(u,v)
            g = A.rotate(C.R_BC,(x,y,1))
            uv = pixel_of((1.5*g[0]/g[2],1.5*g[1]/g[2]),position,true_q,1.5)
            pairs.extend((u,v,*uv))
    # Several gross ORB mismatches exercise robust classification.
    for i in range(0,20,4):
        pairs[4*i+2],pairs[4*i+3] = 45+i, 210-i
    raw = engine.array('f',[0]*23)
    engine.core.recover(render(position,true_q),engine.meta,133333,pairs,raw)
    assert raw[0]==2 and math.dist(raw[1:3],position)<.003, list(raw)
    assert abs(raw[5]-.045)<.003, list(raw)
    assert raw[7]<raw[6], list(raw)
    print('PASS: residual yaw and outlier rejection in native ground geometry')


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
    old=C.LOST_RESET_US
    C.LOST_RESET_US=50000
    for i in range(3):
        stamp+=33333;link.clock.now=stamp
        link.attitudes.append((stamp,(1,0,0,0),(0,0,0),stamp))
        link.ranges.append((stamp,1.5,.05,8,100,None,stamp))
        result=controller.process(Image(WIDTH*HEIGHT),stamp,link)
        assert not result['valid']
    C.LOST_RESET_US=old
    assert result['reset_counter']==1,result
    print('PASS: native/Python controller, histories, display, publish-once and explicit reset')


def check_micropython(executable):
    folder = ROOT/'native'/'_build'
    (folder/'reference.gray').write_bytes(render())
    (folder/'shifted.gray').write_bytes(render((.012,-.009)))
    desc_a=[(u,v) for v in (40,80,120,160,200) for u in (40,90,140,190,240,280)]
    desc_b=[]
    for u,v in desc_a:
        x,y=A.undistort(u,v)
        g=A.rotate(C.R_BC,(x,y,1))
        desc_b.append(pixel_of((1.5*g[0],1.5*g[1]),(.012,-.009),(1,0,0,0),1.5))
    script = '''
import sys
sys.path.insert(0, %r)
from array import array
import rm_vo, algorithm as A, config as C
from main import validate_config
from comm import History
validate_config()
core=rm_vo.Core(A.native_settings())
meta=array('f', C.R_BC+(0,0,0,1.5))
# Exercise 64-bit stamps beyond uint32 microsecond wrap.
stamp=array('I', (100000,1))
prediction=array('f', (0,0,40))
out=array('f', [0]*23)
with open(%r,'rb') as f: reference=bytearray(f.read())
with open(%r,'rb') as f: shifted=bytearray(f.read())
core.step(reference,meta,stamp,prediction,out)
assert out[0]==1, out
reference[:]=bytes(len(reference))
stamp[0]=133333
core.step(shifted,meta,stamp,prediction,out)
assert out[0]==2 and abs(out[1]-.012)<.006 and abs(out[2]+.009)<.006, out
assert abs(out[18]-.033333)<.00001, out
try:
    core.step(b'x',meta,stamp,prediction,out)
    raise AssertionError('short buffer accepted')
except ValueError:
    pass
history=History(3)
for i in range(5): history.append((i,))
assert [x[0] for x in reversed(history)]==[4,3,2]
core.reset()
stamp[0]=166666
core.step(shifted,meta,stamp,prediction,out)
assert out[0]==1
print('PASS: real MicroPython binding, uint64 time, buffers, position and reset')
class Image(bytearray):
    def width(self): return 320
    def height(self): return 240
    def format(self): return 1
    def find_keypoints(self, **kwargs): return self.descriptor
class Match:
    def match(self): return [(i,i) for i in range(30)]
class ImageModule:
    GRAYSCALE=1
    CORNER_FAST=0
    CORNER_AGAST=1
    def match_descriptor(self,*args,**kwargs): return Match()
sys.modules['image']=ImageModule()
ref=Image(open(%r,'rb').read())
current=Image(shifted)
ref.descriptor=%r
current.descriptor=%r
engine=A.FeatureEngine()
assert engine.process(ref,(1,0,0,0),1.5,(0,0,0),100000)['status']==1
class ForceRecovery:
    def __init__(self,core): self.core=core
    def step(self,image,meta,stamp,prediction,out): out[0]=0
    def recover(self,*args): return self.core.recover(*args)
engine.core=ForceRecovery(engine.core)
motion=engine.process(current,(1,0,0,0),1.5,(0,0,0),133333)
assert motion['status']==2 and engine.diagnostics['orb'], motion
assert abs(motion['position'][0]-.012)<.003 and abs(motion['position'][1]+.009)<.003
import test as tests
samples=tests.Samples(8)
samples.add(100)
assert samples.count==1
print('PASS: MicroPython frontend, ORB pair transport and performance observer')
''' % (str(ROOT),str(folder/'reference.gray'),str(folder/'shifted.gray'),
       str(folder/'reference.gray'),desc_a,desc_b)
    subprocess.run([str(executable),'-X','heapsize=4M','-c',script],cwd=ROOT,check=True)


if __name__=='__main__':
    LIB=build()
    sys.modules['rm_vo']=types.SimpleNamespace(Core=HostCore,ABI_VERSION=1)
    sys.modules['image']=types.SimpleNamespace(GRAYSCALE=1,CORNER_FAST=0,CORNER_AGAST=1)
    for p in ROOT.glob('*.py'):
        compile(p.read_text(encoding='utf-8'),str(p),'exec')
    RAYS=[A.undistort(u,v) for v in range(HEIGHT) for u in range(WIDTH)]
    print('C context bytes:',LIB.vo_context_bytes(),flush=True)
    check_frontend()
    check_rotations()
    check_geometry()
    check_controller()
    if '--micropython' in sys.argv:
        check_micropython(sys.argv[sys.argv.index('--micropython')+1])
    print('All PC checks passed. Hardware FPS/firmware build remain separate.')
