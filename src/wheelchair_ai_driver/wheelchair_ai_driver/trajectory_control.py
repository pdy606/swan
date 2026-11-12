"""Bicycle-compatible path following. No obstacle sensors or ROS dependencies."""
from dataclasses import dataclass
import math

from .waypoints import relative_goal
from .observation import wrap_angle


@dataclass
class ControlLimits:
    max_speed: float = 0.30
    wheelbase: float = 0.81
    max_steer: float = math.radians(35)
    max_yaw_rate: float = 0.45
    acceleration: float = 0.15
    deceleration: float = 0.35
    lateral_acceleration: float = 0.20
    steer_rate: float = math.radians(40)
    lookahead: float = 0.65

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('Control limits must be finite and positive')
        if self.max_steer >= math.pi/2:
            raise ValueError('Steering must be less than pi/2')


class PathFollower:
    def __init__(self, limits=None):
        self.limits = limits or ControlLimits()
        self.reset()

    def reset(self):
        self.speed = 0.0
        self.steer = 0.0

    def command(self, path_odom, pose, model_xy, remaining, dt):
        """Follow world-transformed prediction from CURRENT pose, not camera-time pose."""
        if (len(path_odom) != 8 or len(model_xy) != 8 or not 0 < dt <= 0.2
                or not math.isfinite(remaining) or remaining < 0
                or any(len(p) != 2 or not all(math.isfinite(v) for v in p)
                       for p in [*path_odom, *model_xy])):
            raise ValueError('Invalid path/control interval')
        local = [relative_goal(point, pose) for point in path_odom]
        forward = [point for point in local if point[0] > 0.08]
        if not forward:
            self.reset()
            return 0.0, 0.0, {'reason': 'no_forward_prediction'}
        lookahead = self.limits.lookahead + 0.6*self.speed
        target = forward[-1]
        previous = (0.0, 0.0)
        accumulated = 0.0
        for point in forward:
            length = math.dist(previous, point)
            if accumulated + length >= lookahead and length > 1e-9:
                fraction = (lookahead-accumulated)/length
                target = tuple(a+fraction*(b-a) for a, b in zip(previous, point))
                break
            accumulated += length
            previous = point
        x, y = target
        curvature = 2*y/max(x*x+y*y, 0.01)
        wanted_steer = max(-self.limits.max_steer, min(self.limits.max_steer,
                                                    math.atan(self.limits.wheelbase*curvature)))
        self.steer += max(-self.limits.steer_rate*dt,
                          min(self.limits.steer_rate*dt, wanted_steer-self.steer))
        curvature = math.tan(self.steer)/self.limits.wheelbase
        # Match official model's third-waypoint / 2.5s progression estimate. No forced minimum speed.
        progression = max(0.0, model_xy[2][0])/2.5
        heading = math.atan2(y, x)
        desired = min(self.limits.max_speed, progression,
                      math.sqrt(self.limits.lateral_acceleration/max(abs(curvature), 1e-6)),
                      self.limits.max_yaw_rate/max(abs(curvature), 1e-6),
                      math.sqrt(2*self.limits.deceleration*max(0.0, remaining-0.25)))
        desired *= max(0.0, math.cos(heading))**2
        delta = desired-self.speed
        self.speed += max(-self.limits.deceleration*dt, min(self.limits.acceleration*dt, delta))
        if desired < 0.02:
            self.speed = 0.0  # Existing front-drive deadband; never pivot in place.
        return self.speed, self.speed*curvature, {
            'target_local': target, 'steer_rad': self.steer,
            'model_speed': progression, 'desired_speed': desired,
        }


class ReverseRecovery:
    """Gazebo-only staged reverse without rear perception, per experiment policy.

    20cm then 10cm legs, each followed by a zero-command confirmation window.
    The original supervisor alone decides RECOVERED. Limits include all legs and
    waiting; neither a new leg nor WAIT_RECOVERED resets the 15s simulation budget.
    """
    def __init__(self, pose, sim_time, wall_time, distance=0.20,
                 increment=0.10, max_distance=0.50, time_limit=15.0, wait_sec=1.2):
        values=(distance,increment,max_distance,time_limit,wait_sec)
        if any(not math.isfinite(v) or v <= 0 for v in values) or distance > max_distance:
            raise ValueError('Invalid recovery limits')
        if not all(math.isfinite(v) for v in (pose.x,pose.y,pose.yaw,sim_time,wall_time)):
            raise ValueError('Invalid recovery origin')
        self.origin = pose
        self.started = sim_time
        self.wall_started = wall_time
        self.distance = distance
        self.target = distance
        self.increment = increment
        self.max_distance = max_distance
        self.time_limit = time_limit
        self.wait_sec = wait_sec
        self.phase = 'SETTLING'
        self.wait_started = None
        self.reverse_started = None
        self.last_progress = 0.0
        self.last_progress_time = sim_time
        self.backward = 0.0
        self.cumulative_reverse_m = 0.0
        self.last_pose = pose
        self.last_sim = sim_time
        self.legs_started = 0
        self.legs = []
        self.limit_reached = False

    def summary(self, now):
        return {'phase':self.phase,'started_sim':self.started,'elapsed_sim_sec':max(0.,now-self.started),
                'origin':vars(self.origin),'target_from_origin_m':self.target,
                'backward_from_origin_m':self.backward,'cumulative_reverse_m':self.cumulative_reverse_m,
                'additional_reverses':max(0,sum('command_started_sim' in leg for leg in self.legs)-1),
                'legs':[dict(leg) for leg in self.legs],
                'max_distance_m':self.max_distance,'time_limit_sim_sec':self.time_limit,
                'rear_perception_used':False,'simulation_only':True}

    def begin_leg(self, now):
        self.phase = 'REVERSING'
        self.reverse_started = now
        self.last_progress_time = now
        self.last_progress = self.backward
        self.legs_started += 1
        self.legs.append({'index':self.legs_started,'start_sim':now,
                          'start_backward_m':self.backward,'target_m':self.target})

    def step(self, pose, now, wall):
        if not all(math.isfinite(v) for v in (pose.x,pose.y,pose.yaw,now,wall)):
            return 'FAILED',0.,'reverse_odom_deviation'
        if now < self.last_sim or wall < self.wall_started or wall-self.wall_started > 120:
            return 'FAILED',0.,'recovery_clock_or_wall_timeout'
        dx,dy = pose.x-self.origin.x,pose.y-self.origin.y
        backward = -dx*math.cos(self.origin.yaw)-dy*math.sin(self.origin.yaw)
        lateral = -dx*math.sin(self.origin.yaw)+dy*math.cos(self.origin.yaw)
        jump = math.hypot(pose.x-self.last_pose.x,pose.y-self.last_pose.y)
        if (abs(lateral) > .10 or abs(wrap_angle(pose.yaw-self.origin.yaw)) > .25
                or backward < -.06 or jump > max(.10,2*(now-self.last_sim))):
            return 'FAILED',0.,'reverse_odom_deviation'
        elapsed_tick = now-self.last_sim
        self.cumulative_reverse_m += max(0.,backward-self.backward)
        self.backward = backward
        self.last_pose,self.last_sim = pose,now
        if backward > self.max_distance or self.cumulative_reverse_m > self.max_distance:
            return 'FAILED',0.,'recovery_distance_limit'
        if now-self.started >= self.time_limit:
            return 'FAILED',0.,'recovery_sim_timeout'
        if elapsed_tick == 0:
            return self.phase,0.,'recovery_sim_paused'
        if self.phase == 'SETTLING' and now-self.started >= .3:
            self.begin_leg(now)
        if self.phase == 'REVERSING':
            remaining = min(self.target-backward,self.max_distance-backward,
                            self.max_distance-self.cumulative_reverse_m)
            if remaining <= .003:
                self.phase = 'WAIT_RECOVERED'
                self.wait_started = now
                self.limit_reached = (self.max_distance-self.cumulative_reverse_m <= .003
                                      or self.max_distance-backward <= .003)
                self.legs[-1].update({'stop_sim':now,'stop_backward_m':backward,
                                     'cumulative_reverse_m':self.cumulative_reverse_m})
            else:
                if backward-self.last_progress >= .01:
                    self.last_progress,self.last_progress_time = backward,now
                if now-self.reverse_started > 4.0 or now-self.last_progress_time > 1.5:
                    return 'FAILED',0.,'reverse_stalled_or_timeout'
                # Slow near each leg target to reduce coast beyond the 50cm cap.
                self.legs[-1].setdefault('command_started_sim',now)
                return self.phase,-min(.10,max(.03,remaining/.20)),'reverse'
        if self.phase == 'WAIT_RECOVERED' and now-self.wait_started >= self.wait_sec:
            if self.limit_reached or self.target >= self.max_distance-1e-9:
                return 'FAILED',0.,'recovery_distance_limit_without_recovered'
            self.target = min(self.max_distance,self.target+self.increment)
            self.begin_leg(now)
            # Keep this transition tick at zero; reverse begins on the next tick.
            return self.phase,0.,'additional_reverse_prepared'
        return self.phase,0.,'waiting'
