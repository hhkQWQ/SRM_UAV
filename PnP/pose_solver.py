"""固定四点平面PnP：归一化单应初始化 + 有界LM。只依赖math/config。"""
import math
import config as C

def finite(values):
    return all(math.isfinite(v) for v in values)

def hypot2(x, y):
    return math.sqrt(x*x+y*y)

def dot(a, b):
    return sum(a[i]*b[i] for i in range(len(a)))

def norm(a):
    return math.sqrt(dot(a, a))

def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

def unit(a):
    n = norm(a)
    if n < 1e-12 or not math.isfinite(n):
        raise ValueError("zero vector")
    return tuple(v/n for v in a)

def mv(r, p):
    return tuple(dot(row, p) for row in r)

def transpose(r):
    return tuple(tuple(r[j][i] for j in range(3)) for i in range(3))

def mm(a, b):
    bt = transpose(b)
    return tuple(tuple(dot(row, col) for col in bt) for row in a)

def quaternion_rotation(q):
    if len(q) != 4 or not finite(q) or abs(norm(q)-1.) > 0.15:
        raise ValueError("invalid quaternion")
    w, x, y, z = unit(q)
    return ((1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)),
            (2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)),
            (2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)))

def rotation_valid(r):
    try:
        return (len(r) == 3 and all(len(row) == 3 and finite(row) for row in r)
                and all(abs(dot(r[i], r[j])-(1. if i == j else 0.)) < 0.01
                        for i in range(3) for j in range(3))
                and dot(cross(r[0], r[1]), r[2]) > 0.99)
    except (TypeError, ValueError):
        return False

def camera_to_body(p, r_bc, t_bc):
    p = mv(r_bc, p)
    return tuple(p[i]+t_bc[i] for i in range(3))

def local_to_camera(p, q_nb, r_bc, t_bc):
    body = mv(transpose(quaternion_rotation(q_nb)), p)
    return mv(transpose(r_bc), tuple(body[i]-t_bc[i] for i in range(3)))

def quad_valid(points, min_area=0., min_edge=0.):
    if len(points) != 4 or not all(len(p) == 2 and finite(p) for p in points):
        return False
    area = 0.
    for i in range(4):
        a, b, c = points[i], points[(i+1)%4], points[(i+2)%4]
        dx, dy = b[0]-a[0], b[1]-a[1]
        if dx*dx+dy*dy < min_edge*min_edge:
            return False
        if dx*(c[1]-b[1])-dy*(c[0]-b[0]) <= 1e-12:
            return False
        area += a[0]*b[1]-a[1]*b[0]
    return area*0.5 >= min_area

def linear_solve(a, b):
    """小矩阵带部分主元消元；输入可被修改。"""
    n = len(b)
    for k in range(n):
        pivot = max(range(k, n), key=lambda j: abs(a[j][k]))
        if abs(a[pivot][k]) < 1e-12:
            raise ValueError("singular system")
        a[k], a[pivot] = a[pivot], a[k]
        b[k], b[pivot] = b[pivot], b[k]
        inv = 1./a[k][k]
        for j in range(k, n):
            a[k][j] *= inv
        b[k] *= inv
        for i in range(k+1, n):
            f = a[i][k]
            for j in range(k+1, n):
                a[i][j] -= f*a[k][j]
            b[i] -= f*b[k]
    x = [0.]*n
    for i in range(n-1, -1, -1):
        x[i] = b[i]-sum(a[i][j]*x[j] for j in range(i+1, n))
    if not finite(x):
        raise ValueError("nonfinite system")
    return x

def exp_rotation(w):
    angle = norm(w)
    s = 1. if angle < 1e-8 else math.sin(angle)/angle
    c = 0.5 if angle < 1e-8 else (1-math.cos(angle))/(angle*angle)
    x, y, z = w
    return ((1-c*(y*y+z*z), c*x*y-s*z, c*x*z+s*y),
            (c*x*y+s*z, 1-c*(x*x+z*z), c*y*z-s*x),
            (c*x*z-s*y, c*y*z+s*x, 1-c*(x*x+y*y)))

class PlanarPoseSolver:
    def __init__(self, points, intrinsics=None, distortion=None, iterations=None):
        if points is None or len(points) != 4:
            raise ValueError("fill TARGET_POINTS_T_M with four measured points")
        if not all(len(p) == 3 and finite(p) and abs(p[2]) < 1e-8 for p in points):
            raise ValueError("model must be metric points on Z_T=0")
        xy = tuple((float(p[0]), float(p[1])) for p in points)
        if not quad_valid(xy) or norm(tuple(sum(p[k] for p in points)/4. for k in range(3))) > 1e-5:
            raise ValueError("model order/centre invalid; use 1,2,3,4 about centre")
        # 对角线也必须通过定义的物理中心；防止错误四点模型被当作中心原点。
        if norm(tuple(points[0][i]+points[2][i] for i in range(3))) > 1e-5 or norm(tuple(points[1][i]+points[3][i] for i in range(3))) > 1e-5:
            raise ValueError("opposite points must be symmetric about physical centre")
        self.xy = xy
        self.k = tuple(C.INTRINSICS if intrinsics is None else intrinsics)
        self.d = tuple(C.DIST_COEFFS if distortion is None else distortion)
        if len(self.k) != 4 or not finite(self.k) or min(self.k[:2]) <= 0 or len(self.d) != 5 or not finite(self.d):
            raise ValueError("invalid calibration")
        self.iterations = C.REFINE_ITERATIONS if iterations is None else iterations
        self.obj_scale = math.sqrt(2.)/(sum(hypot2(*p) for p in xy)*0.25)

    def undistort(self, p):
        fx, fy, cx, cy = self.k
        xd, yd = (p[0]-cx)/fx, (p[1]-cy)/fy
        x, y = xd, yd
        k1, k2, p1, p2, k3 = self.d
        for _ in range(C.UNDISTORT_ITERATIONS):
            r2 = x*x+y*y
            radial = 1+r2*(k1+r2*(k2+r2*k3))
            if abs(radial) < 1e-8:
                raise ValueError("distortion singular")
            x, y = ((xd-2*p1*x*y-p2*(r2+2*x*x))/radial,
                    (yd-p1*(r2+2*y*y)-2*p2*x*y)/radial)
        if not finite((x, y)) or max(abs(x), abs(y)) > 5:
            raise ValueError("undistortion failed")
        return x, y

    def project(self, p):
        if p[2] <= 1e-6 or not finite(p):
            return None
        x, y = p[0]/p[2], p[1]/p[2]
        r2 = x*x+y*y
        k1, k2, p1, p2, k3 = self.d
        radial = 1+r2*(k1+r2*(k2+r2*k3))
        xd = x*radial+2*p1*x*y+p2*(r2+2*x*x)
        yd = y*radial+p1*(r2+2*y*y)+2*p2*x*y
        fx, fy, cx, cy = self.k
        return fx*xd+cx, fy*yd+cy

    def _initial(self, image):
        mu, mv_ = (sum(p[k] for p in image)*0.25 for k in (0, 1))
        radius = sum(hypot2(p[0]-mu, p[1]-mv_) for p in image)*0.25
        if radius < 1e-8:
            raise ValueError("tiny projection")
        si, so = math.sqrt(2.)/radius, self.obj_scale
        a, b = [], []
        for (ox, oy), (ix, iy) in zip(self.xy, image):
            x, y, u, v = ox*so, oy*so, (ix-mu)*si, (iy-mv_)*si
            a.append([x,y,1.,0.,0.,0.,-u*x,-u*y]); b.append(u)
            a.append([0.,0.,0.,x,y,1.,-v*x,-v*y]); b.append(v)
        h = linear_solve(a, b)
        hn = (tuple(h[:3]), tuple(h[3:6]), (h[6],h[7],1.))
        h = mm(((1/si,0.,mu),(0.,1/si,mv_),(0.,0.,1.)),
               mm(hn, ((so,0.,0.),(0.,so,0.),(0.,0.,1.))))
        c1, c2, c3 = transpose(h)
        scale = 2./(norm(c1)+norm(c2))
        if c3[2] < 0:
            scale = -scale
        r1 = unit(tuple(scale*v for v in c1))
        v2 = tuple(scale*v for v in c2)
        along = dot(r1, v2)
        r2 = unit(tuple(v2[i]-along*r1[i] for i in range(3)))
        r = transpose((r1, r2, cross(r1,r2)))
        return r, tuple(scale*v for v in c3)

    def _cost(self, r, t, observations, jacobian=False):
        a, b, cost = [[0.]*6 for _ in range(6)], [0.]*6, 0.
        fx, fy = self.k[:2]
        for (x, y), obs in zip(self.xy, observations):
            v = mv(r, (x,y,0.))
            px, py, pz = (v[i]+t[i] for i in range(3))
            if pz <= 1e-6:
                raise ValueError("behind camera")
            xn, yn = px/pz, py/pz
            ru, rv = (obs[0]-xn)*fx, (obs[1]-yn)*fy
            cost += ru*ru+rv*rv
            if jacobian:
                dv = ((0.,-v[2],v[1]),(v[2],0.,-v[0]),(-v[1],v[0],0.))
                ju = [fx*(d[0]-xn*d[2])/pz for d in dv]+[fx/pz,0.,-fx*xn/pz]
                jv = [fy*(d[1]-yn*d[2])/pz for d in dv]+[0.,fy/pz,-fy*yn/pz]
                for i in range(6):
                    b[i] += ju[i]*ru+jv[i]*rv
                    for j in range(i,6):
                        a[i][j] += ju[i]*ju[j]+jv[i]*jv[j]
        if jacobian:
            for i in range(6):
                for j in range(i):
                    a[i][j] = a[j][i]
        return cost, a, b

    def solve(self, pixels, previous=None):
        if not quad_valid(pixels, C.MIN_QUAD_AREA_PX2, C.MIN_EDGE_PX):
            return {"valid":False, "reason":"POINT_GEOMETRY"}
        try:
            observations = tuple(self.undistort(p) for p in pixels)
            r, t = self._initial(observations)
            cost = self._cost(r,t,observations)[0]
            if previous is not None and previous.get("valid"):
                pr, pt = previous["rotation"], previous["position"]
                pcost = self._cost(pr,pt,observations)[0]
                if pcost < cost:
                    r,t,cost = pr,pt,pcost
            damping, used = 1e-3, 0
            for _ in range(self.iterations):
                if cost < 1e-10:
                    break
                _, a, b = self._cost(r,t,observations,True)
                for i in range(6):
                    a[i][i] += damping*(a[i][i]+1e-6)
                step = linear_solve(a,b)
                if norm(step[:3]) > 0.5 or norm(step[3:]) > max(0.1,t[2]):
                    damping *= 10.
                    continue
                trial_r = mm(exp_rotation(step[:3]),r)
                trial_t = tuple(t[i]+step[i+3] for i in range(3))
                try:
                    trial_cost = self._cost(trial_r,trial_t,observations)[0]
                except ValueError:
                    trial_cost = float("inf")
                used += 1
                if trial_cost < cost:
                    r,t,cost = trial_r,trial_t,trial_cost
                    damping = max(1e-8,damping*0.3)
                    if norm(step) < 1e-7:
                        break
                else:
                    damping *= 10.
            error = 0.
            for (x,y), observed in zip(self.xy,pixels):
                rotated = mv(r,(x,y,0.))
                p = tuple(rotated[i]+t[i] for i in range(3))
                uv = self.project(p)
                if uv is None:
                    raise ValueError("negative depth")
                error += (uv[0]-observed[0])**2+(uv[1]-observed[1])**2
            rms = math.sqrt(error/4.)
            valid = (finite(t) and C.MIN_DEPTH_M <= t[2] <= C.MAX_DEPTH_M
                     and math.isfinite(rms) and rms <= C.MAX_REPROJECTION_PX)
            return {"valid":valid, "reason":"OK" if valid else "POSE_QUALITY",
                    "position":t, "rotation":r, "reprojection_px":rms,
                    "center":self.project(t), "iterations":used}
        except (ValueError, ZeroDivisionError, OverflowError):
            return {"valid":False, "reason":"NUMERICAL"}

def relative_measurement(pose, q_nb, r_bc, t_bc):
    """水平朝上标靶：T的+z指向板内，局部Down应与该轴一致。"""
    if not pose or not pose.get("valid") or q_nb is None or t_bc is None:
        return None
    if not rotation_valid(r_bc) or len(t_bc) != 3 or not finite(t_bc):
        return None
    r_nb = quaternion_rotation(q_nb)
    p = mv(r_nb,camera_to_body(pose["position"],r_bc,t_bc))
    normal = mv(r_nb,mv(r_bc,tuple(row[2] for row in pose["rotation"])))
    if normal[2] < math.cos(C.MAX_NORMAL_TILT_DEG*math.pi/180.) or p[2] <= C.MIN_DEPTH_M:
        return None
    return p
