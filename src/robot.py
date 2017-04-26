import numpy as np
import pygame as pg
import pygame.gfxdraw
from window import Window
from particle import Particle
from math import atan2

eps = 1e-31
s = np.sin
c = np.cos


class Robot():
    """
    Object simulating the differential drive robot
    """

    def __init__(self, width, wheelRadius, mass=1, pos=[0, 0], ang=90,
                 vel=0, omg=0, torquePulse=2000, window=Window(), dt=1):

        self.dt = dt

        # Dynamic values
        self.torque = np.array([0, 0], dtype=np.float64)
        self.torquePulse = torquePulse
        self.friction_const = 0.1
        self.BRAKE = False

        # Pose
        self.pos = np.array(pos, dtype=np.float64)
        self.ang = np.radians(ang)

        # Velocities
        self.vel = 0
        self.omg = omg

        # Properties
        self.wheelRadius = np.array(wheelRadius, dtype=np.float64)
        self.width = width
        self.mass = mass
        self.inertia = mass*(
            (1+wheelRadius**2)/6+(width/2+0.5)**2/12 + (
            (width**2 + (wheelRadius + 1)**2)/12 + 1/4 + width**2 + 1 + 4))
        self.wheel_inertia = (mass/5)*self.wheelRadius**2
        self.max_torque = torquePulse*2

        # Graphic vertices {PART: [[vertices,] , (R, G, B)]}
        self.verts = {
            "BODY": [np.array([
                [-width/2, -wheelRadius],           # Bottom_left
                [width/2, -wheelRadius],            # Bottom_right
                [width/2, wheelRadius + 1],         # Top_right
                # [0, wheelRadius + 2],              X Tip
                [-width/2, wheelRadius + 1]],       # Top_left
                dtype=np.float64),
                (255, 100, 100)],                   # Salmon
            "LWHEEL": [np.array([
                [-width/2, -wheelRadius],           # Bottom_right
                [-width/2, wheelRadius],            # Top_right
                [-width/2 - 1, wheelRadius],        # Top_left
                [-width/2 - 1, -wheelRadius]],      # Bottom_left
                dtype=np.float64),
                (0, 150, 255)],                     # Aqua
            "RWHEEL": [np.array([
                [width/2, -wheelRadius],            # Bottom_left
                [width/2, wheelRadius],             # Top_left
                [width/2 + 1, wheelRadius],         # Top_right
                [width/2 + 1, -wheelRadius]],       # Bottom_right
                dtype=np.float64),
                (0, 150, 255)],                     # Aqua
            "PAIL": [np.array([
                [1.8*width/3,   wheelRadius + 1],
                [2*width/3,   wheelRadius + 2],
                [1.8*width/3, wheelRadius + 2],
                [1.5*width/3, wheelRadius + 1.2],
                [-1.5*width/3, wheelRadius + 1.2],
                [-1.8*width/3, wheelRadius + 2],
                [-2*width/3,   wheelRadius + 2],
                [-1.8*width/3,   wheelRadius + 1],
            ]),
                (200, 0, 255)]}

        # Window used for mappings to screen-space
        self.window = window

        # Center, where self.pos should effectively be positioned
        self.center = Centroid(self.verts["BODY"][0])

        # Initial positioning
        self.transform(Translate, vector=np.negative(self.center))
        self.basis = np.array([[1, 0], [0, 1]], dtype=np.float64)

        # Projectile
        self.projectile = None

        # Automatic control variables
        self.AUTO = False
        self.turning = False
        self.moving = False
        self.move_finished = True
        self.objective = None
        self.poly_step = 0

    def update(self):
        """
        
        """

        dt = self.dt
        th = self.ang
        m = self.mass
        R = self.wheelRadius
        I = self.inertia
        Iw = self.wheel_inertia
        L = self.width/2

        if self.AUTO and self.objective is not None:
            self.auto_control()

        Tr, Tl = self.torque

        # Integrate torque according to dynamical model
        self.vel += (((Tr+Tl)/R))/(m+2*Iw**2/R**2)*dt
        self.omg += ((L*(Tr-Tl)/R))/(I+2*Iw*L**2/R**2)*dt

        # Dampening due to friction forces
        self.vel *= 1 - self.friction_const
        self.omg *= 1 - self.friction_const

        # Integrate velocities into position
        self.pos += self.vel*dt*np.array([np.cos(th), np.sin(th)])
        self.ang += self.omg*dt

        # Update vertices
        self.transform(Translate, vector=self.pos-self.center)
        self.transform(Rotate, theta=self.omg*dt, axis=self.center)

        # Update center and basis
        self.center = Centroid(self.verts["BODY"][0])
        self.basis = Rotate(self.basis, self.omg*dt)

        # Update and check projectile
        if self.projectile is not None:

            # Collision with objective ball
            if self.objective is not None:
                r = self.projectile.size + self.width/4
                if np.linalg.norm(self.objective - self.projectile.pos) < r:
                    self.objective = None
                    return

            if not self.projectile.update(dt):
                self.projectile = None

    def add_torque(self, Lt, Rt):
        self.torque += np.array([Lt, Rt], dtype=np.float64)
        self.torque = np.clip(self.torque, -self.max_torque, self.max_torque)

    def transform(self, function, **kwargs):
        for poly in self.verts:
            self.verts[poly][0] = function(self.verts[poly][0], **kwargs)

    def draw(self, surface):

        # Localizing variables to avoid collisions in parallelism
        mapall_ = self.window.mapall_
        map_ = self.window.map_
        body_parts = self.verts.values()
        projectile = self.projectile
        objective = self.objective

        for part in body_parts:
            # part[0]: Vertices
            # part[1]: Color
            # For some reason I can't use "for vertices, color in part" O.o
            try:
                pg.gfxdraw.aapolygon(surface,  mapall_(part[0]), part[1])
                pg.gfxdraw.filled_polygon(surface,  mapall_(part[0]), part[1])
            except:
                pg.draw.polygon(surface, part[1], mapall_(part[0]))

        # Draw projectile
        if projectile is not None:
            radius = int(self.window.alpha_*projectile.size)
            pg.draw.circle(surface, projectile.color,
                           map_(*projectile.pos), radius)

        # Draw objective ball
        if objective is not None:
            radius = int(self.window.alpha_*self.width/4)
            try:
                pg.gfxdraw.aacircle(surface, map_(*objective),
                                    radius, (100, 200, 0))
            except:
                pg.draw.circle(surface, (100, 200, 0),
                               map_(*objective), radius)

    def reset(self):
        self.transform(Rotate, theta=-self.ang, axis=self.center)
        self.basis = Rotate(self.basis, -self.ang)
        self.pos *= 0
        self.vel *= 0
        self.ang = 0
        self.omg *= 0
        self.AUTO = False
        self.objective = None
        self.projectile = None

    def brake(self, BRAKE):
        self.BRAKE = True

    def fire(self):
        if self.projectile is None:
            proj_pos = self.pos + self.basis[1]*(self.wheelRadius + 1.5)
            proj_vel = 50
            self.projectile = Particle(
                self.window, proj_pos, self.width/2,
                np.dot(np.array([0, proj_vel]), self.basis))

    def wasd_manipulate(self, F, B, R, L):
        # Movement scheme:
        #   On forwards or backwards, right or left ADD velocity for
        #   'smooth' curves. If only R or L, opposite wheel will spin
        #   backwards so as to spin in place.
        tq = self.torquePulse
        self.add_torque(
            tq*((F - B)*(1 + L*(not R)) + (L - R)*(not (F or B))),
            tq*((F - B)*(1 + R*(not L)) + (R - L)*(not (F or B)))
        )

    def goal_angle(self, objective_pos):
        v1 = objective_pos - self.pos
        v2 = self.basis[1]
        norm = np.linalg.norm
        dot = np.dot(v1, v2)
        sense = -np.sign(np.cross(v1, v2))
        # angle = atan2(norm(np.cross(v1, v2)), np.dot(v1, v2))
        angle = np.arccos(dot/(norm(v1)*norm(v2)))
        return angle, sense

    def goal_distance(self, objective_pos):
        return np.linalg.norm(objective_pos - self.pos)

    def auto_control(self):

        dt = self.dt
        angle, sense = self.goal_angle(self.objective)
        dist = self.goal_distance(self.objective)

        # Move_finished is used to reset a t value for poly interpolation

        # Acceptable margin of error for goal to be reached
        a_margin = 0.1 / dist
        d_margin = 0.5

        if angle > a_margin:
            if self.move_finished:
                self.turning = True
                self.move_finished = False
        elif self.turning:
            self.turning = False
            self.move_finished = True
            self.poly_step = 0

        if dist > d_margin:
            if self.move_finished:
                self.moving = True
                self.move_finished = False
        elif self.moving:
            self.moving = False
            self.move_finished = True
            self.poly_step = 0

        t = self.poly_step * self.dt

        if self.turning:
            tf = 0.1
            tr, tl = self.calc_torque(0, sense*angle, t, tf)
            # sense?
            self.add_torque(tr, tl)
        elif self.moving:
            tf = 0.1
            tr, tl = self.calc_torque(dist, 0, t, tf)
            # sense?
            self.add_torque(tr, tl)
        else:
            self.objective = None

        self.poly_step += 1

    def calc_torque(self, dist, ang, t, tf):
        # Equations calculated using cubic interpolation and
        # ecuations of motion (see report)
        Iw = self.wheel_inertia
        I = self.inertia
        L = self.width/2
        m = self.mass
        R = self.wheelRadius
        c = self.friction_const

        T1 = 6*dist*Iw/R*tf**2 + 3*dist*m*R/tf**2-12*dist*Iw*t/(R*tf**3)
        T2 = 12*ang*Iw*L*t/(R*tf**3) + 6*ang*I*R*t/(L*tf**3) -6*ang*Iw*L/(R*tf**2)-3*ang*I*R/(L*tf**2)
        f = -c*Iw/R - c*Iw*L/R - c*I/(2*L) - c*m*R/2

        Tr = T1 - T2 - f
        Tl = T1 + T2 - f
        return -Tr, -Tl


    def __str__(s):
        s = """
ROBOT ID: %f
Position: (%0.2f, %0.2f)
Velocity: %0.2f
Orientation: %0.2f

IsAutomatic: %i
HasFired: %i
        """ % (id(s), *s.pos, s.vel, s.ang,
               int(s.AUTO), int(s.projectile is not None))
        return(s)

    def gather_data(self):
        bodies = self.verts


def Rot2D(vector, theta):
    c = np.cos(theta)
    s = np.sin(theta)
    m = np.matrix([[c, -s], [s, c]])
    return (np.dot(m, vector)).A[0]


def Centroid(poly_points):
    return np.array([
        np.mean(poly_points[:, 0], dtype=np.float64),
        np.mean(poly_points[:, 1], dtype=np.float64)
    ])


def Translate(poly_points, vector):
    new_points = np.empty_like(poly_points)
    for i in range(len(poly_points)):
        new_points[i] = poly_points[i] + np.array(vector)
    return new_points


def Rotate(poly_points, theta, axis=(0, 0)):
    new_points = np.empty_like(poly_points)
    for i in range(len(poly_points)):
        new_points[i] = Rot2D(poly_points[i]-axis, theta) + axis
    return new_points


def draw_poly(surf, points, color):
    pg.draw.polygon(surf, color, [map_(i, j) for i, j in points])