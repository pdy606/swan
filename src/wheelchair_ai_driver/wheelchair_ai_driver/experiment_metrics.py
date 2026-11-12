"""Ordered course progress, independent of odometry distance and obstacle sensing."""
import math


class CourseProgress:
    def __init__(self, origin, waypoints, corridor=2.0):
        self.points=[tuple(origin),*map(tuple,waypoints)]
        self.lengths=[math.dist(a,b) for a,b in zip(self.points,self.points[1:])]
        if not self.lengths or min(self.lengths)<=0:
            raise ValueError('Course segments must have positive length')
        self.total=sum(self.lengths)
        self.segment=0
        self.furthest=0.0
        self.corridor=corridor
        self.arrived=False

    def update(self, point, arrived=False):
        if not all(math.isfinite(v) for v in point):
            raise ValueError('Nonfinite pose')
        # Visit each guidepoint in order; a later segment cannot be selected by proximity.
        while self.segment<len(self.lengths)-1 and math.dist(point,self.points[self.segment+1])<=.8:
            self.furthest=max(self.furthest,sum(self.lengths[:self.segment+1]))
            self.segment+=1
        a,b=self.points[self.segment:self.segment+2]
        length=self.lengths[self.segment]
        t=max(0.,min(1.,sum((p-x)*(y-x) for p,x,y in zip(point,a,b))/length**2))
        projection=tuple(x+t*(y-x) for x,y in zip(a,b))
        if math.dist(point,projection)<=self.corridor:
            self.furthest=max(self.furthest,sum(self.lengths[:self.segment])+t*length)
        if arrived and self.segment==len(self.lengths)-1 and math.dist(point,self.points[-1])<=.5:
            self.arrived=True
        return 100. if self.arrived else min(99.999,100*self.furthest/self.total)
