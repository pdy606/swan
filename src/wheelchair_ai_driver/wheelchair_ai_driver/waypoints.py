"""Predeclared route geometry using only pose; no obstacle or planner information."""
import math


def world_to_odom(point, translation=(0.0, 0.0), yaw=0.0):
    """Apply T_odom_world (configuration), not an assumed spawn-origin offset."""
    x, y = point
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (translation[0] + cosine*x - sine*y,
            translation[1] + sine*x + cosine*y)


def relative_goal(point, pose):
    dx, dy = point[0] - pose.x, point[1] - pose.y
    cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
    return cosine*dx + sine*dy, -sine*dx + cosine*dy


class RouteTracker:
    def __init__(self, points, waypoint_radius=0.8, goal_radius=0.5, lookahead=3.0):
        if not points or any(not math.isfinite(value) or value <= 0
                             for value in (waypoint_radius, goal_radius, lookahead)):
            raise ValueError('A route and positive tolerances are required')
        self.points = [tuple(map(float, point)) for point in points]
        if any(len(point) != 2 or not all(math.isfinite(v) for v in point)
               for point in self.points):
            raise ValueError('Each route point must be two finite numbers')
        self.waypoint_radius = waypoint_radius
        self.goal_radius = goal_radius
        self.lookahead = lookahead
        self.index = 0

    def reset(self):
        self.index = 0

    def target(self, pose):
        while self.index < len(self.points) - 1:
            point = self.points[self.index]
            if math.hypot(point[0] - pose.x, point[1] - pose.y) > self.waypoint_radius:
                break
            self.index += 1
        point = self.points[self.index]
        distance = math.hypot(point[0] - pose.x, point[1] - pose.y)
        reached = self.index == len(self.points) - 1 and distance <= self.goal_radius
        scale = min(1.0, self.lookahead / distance) if distance > 0 else 0.0
        temporary = (pose.x + scale * (point[0] - pose.x),
                     pose.y + scale * (point[1] - pose.y))
        return temporary, reached
