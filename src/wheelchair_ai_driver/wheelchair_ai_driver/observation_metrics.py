"""Measurement geometry only. Never used as obstacle input to the driver."""
import math


def route_distance(point, route):
    if len(route)<2:
        return math.dist(point,route[0]) if route else 0.0
    distances=[]
    for a,b in zip(route,route[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        norm=dx*dx+dy*dy
        t=max(0.0,min(1.0,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/norm)) if norm else 0.0
        distances.append(math.dist(point,(a[0]+t*dx,a[1]+t*dy)))
    return min(distances)


class MotionMetrics:
    def __init__(self):
        self.previous=None
        self.distance=0.0
        self.elapsed=0.0
        self.speed_integral=0.0
        self.max_speed=0.0
        self.max_yaw_rate=0.0
        self.odom_jumps=0

    def add(self, stamp,x,y,v,w):
        if not all(math.isfinite(n) for n in (stamp,x,y,v,w)):
            return False
        if self.previous:
            t0,x0,y0,v0=self.previous
            dt=stamp-t0
            if dt<=0:
                return False
            distance=math.hypot(x-x0,y-y0)
            if distance>max(.1,2*dt):
                self.odom_jumps+=1
            else:
                self.distance+=distance
            self.speed_integral+=(abs(v0)+abs(v))*.5*dt
            self.elapsed+=dt
        self.previous=(stamp,x,y,v)
        self.max_speed=max(self.max_speed,abs(v))
        self.max_yaw_rate=max(self.max_yaw_rate,abs(w))
        return True

    def summary(self):
        return {'odom_path_length_m':self.distance,'odom_elapsed_sim_sec':self.elapsed,
                'mean_abs_speed_mps':self.speed_integral/self.elapsed if self.elapsed else 0.0,
                'max_abs_speed_mps':self.max_speed,'max_abs_yaw_rate_radps':self.max_yaw_rate,
                'odom_jumps':self.odom_jumps}
