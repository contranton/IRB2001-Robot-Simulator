import numpy as np
import pygame as pg
import pygame.gfxdraw

from window import Window
from particle import Particle
from math import atan2
from sys import stdout

eps = 1e-31
s = np.sin
c = np.cos

total_steps = 0

class Robot():
    """
    Object simulating the differential drive robot
    """

    def __init__(self, pg_comm_pipe, width, wheelRadius, mass=1, pos=[0, 0], ang=90,
                 vel=0, omg=0, torquePulse=1000, window=Window(), dt=1):

        self.dt = dt
        self.time = 0
        self.pg_comm_pipe = pg_comm_pipe

        # Dynamic values
        self.torque = np.array([0, 0], dtype=np.float32)
        self.torquePulse = torquePulse
        self.friction_const = 0.1
        self.BRAKE = False

        # Pose
        self.pos = np.array(pos, dtype=np.float32)
        self.ang = np.radians(ang)

        # Velocities
        self.vel = 0.0
        self.omg = omg

        # Properties
        self.wheelRadius = np.array(wheelRadius, dtype=np.float32)
        self.width = width
        self.mass = mass
        self.inertia = (mass*((1 + wheelRadius**2)/6 + (width/2 + 0.5)**2/12 +
                        ((width**2 + (wheelRadius + 1)**2)/12 + 1/4 +
                        width**2 + 1 + 4)))
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
                dtype=np.float32),
                (255, 100, 100)],                   # Salmon
            "LWHEEL": [np.array([
                [-width/2, -wheelRadius],           # Bottom_right
                [-width/2, wheelRadius],            # Top_right
                [-width/2 - 1, wheelRadius],        # Top_left
                [-width/2 - 1, -wheelRadius]],      # Bottom_left
                dtype=np.float32),
                (0, 150, 255)],                     # Aqua
            "RWHEEL": [np.array([
                [width/2, -wheelRadius],            # Bottom_left
                [width/2, wheelRadius],             # Top_left
                [width/2 + 1, wheelRadius],         # Top_right
                [width/2 + 1, -wheelRadius]],       # Bottom_right
                dtype=np.float32),
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
        self.basis = np.array([[1, 0], [0, 1]], dtype=np.float32)

        # Projectile
        self.projectile = None

        # Manual control variables
        self.FBRL = 0b0

        # Automatic control variables
        self.AUTO = False
        self.turning = False
        self.moving = False
        self.move_finished = True
        self.objective = None
        self.poly_step = 0

        # Used for closing process
        self.END = False

        # Functions activated by pipe signals sent by pygame
        self.mp_dict = {'WASD': self.update_wasd,
                        'FIRE': self.fire,
                        'TOG_AUTO': self.toggle_auto,
                        'BRAKE_ON': self.brake_on,
                        'BRAKE_OFF': self.brake_off,
                        'OBJ_UPDATE': self.objective_update,
                        'PROJ_DEAD': self.kill_projectile,
                        'RESET': self.reset,
                        'PLS_SEND': self.send_draw_data,
                        'END?': self.toggle_END}

    def toggle_auto(self, *args):
        self.AUTO = not self.AUTO

    def toggle_END(self, *args):
        self.END = True

    def kill_projectile(self, *args):
        self.projectile = None

    def update_wasd(self, vals, *args):
        self.FBRL = vals

    def update(self):

        self.time += self.dt
        self.parse_signal()
        self.torque *= 0

        dt = self.dt
        th = self.ang
        m = self.mass
        R = self.wheelRadius
        I = self.inertia
        Iw = self.wheel_inertia
        L = self.width/2

        if self.AUTO and self.objective is not None:
            self.auto_control()
        else:
            self.wasd_manipulate()
            # print(self.F, self.B, self.R, self.L)
            # self.update_wasd(0b0000)

        if self.BRAKE:
            self.vel *= 1 - 2 * self.friction_const
            self.torque *= 0

        Tr, Tl = self.torque
        if Tr != 0 and Tl != 0:
            # Integrate torque according to dynamical model
            self.vel += (((Tr+Tl)/R))/(m+2*Iw**2/R**2)*dt
            self.omg += ((L*(Tr-Tl)/R))/(I+2*Iw*L**2/R**2)*dt

        # Dampening due to friction forces
        self.vel *= 1 - self.friction_const
        self.omg *= 1 - self.friction_const

        if self.vel != 0 or self.omg != 0:
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
        self.torque += np.array([Lt, Rt], dtype=np.float32)
        self.torque = np.clip(self.torque, -self.max_torque, self.max_torque)

    def transform(self, function, **kwargs):
        for poly in self.verts:
            self.verts[poly][0] = function(self.verts[poly][0], **kwargs)

    def reset(self, *args):
        self.transform(Rotate, theta=-self.ang, axis=self.center)
        self.basis = Rotate(self.basis, -self.ang)
        self.pos *= 0
        self.vel *= 0
        self.ang = 0
        self.omg *= 0
        self.AUTO = False
        self.objective = None
        self.projectile = None

    def brake_on(self, *args):
        self.BRAKE = True

    def brake_off(self, *args):
        self.BRAKE = False

    def fire(self, *args):
        if self.projectile is None:
            proj_pos = self.pos + self.basis[1]*(self.wheelRadius + 1.5)
            proj_vel = 50
            self.projectile = Particle(
                proj_pos, self.width/2,
                np.dot(np.array([0, proj_vel]), self.basis))

    def wasd_manipulate(self, *args):
        # Movement scheme:
        #   On forwards or backwards, right or left ADD velocity for
        #   'smooth' curves. If only R or L, opposite wheel will spin
        #   backwards so as to spin in place.
        FBRL = self.FBRL
        F = (FBRL & 0b1000) >> 3
        B = (FBRL & 0b0100) >> 2
        R = (FBRL & 0b0010) >> 1
        L = (FBRL & 0b0001)
        if F == B == R == L == 0:
            return
        tq = self.torquePulse
        self.add_torque(
            tq*((F - B)*(1 + 2*L*(not R)) + (L - R)*(not (F or B))),
            tq*((F - B)*(1 + 2*R*(not L)) + (R - L)*(not (F or B)))
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

        angle, sense = self.goal_angle(self.objective)
        dist = self.goal_distance(self.objective)

        # Move_finished is used to reset a t value for poly interpolation

        # Acceptable margin of error for goal to be reached
        a_margin = 0.01 / (5*dist)
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
            tf = 0.5
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
        # Equations calculated using quintic interpolation (so that
        # acceleration at beginning and end are 0 and vary smoothly)
        # equations of motion (see report)
        Iw = self.wheel_inertia
        It = self.inertia
        L = self.width/2
        m = self.mass
        R = self.wheelRadius
        c = self.friction_const

        # Cubic
        # T1 = 6*dist*Iw/R*tf**2 + 3*dist*m*R/tf**2-12*dist*Iw*t/(R*tf**3)
        # T2 = 12*ang*Iw*L*t/(R*tf**3) + 6*ang*I*R*t/(L*tf**3) -6*ang*Iw*L/(R*tf**2)-3*ang*I*R/(L*tf**2)
        # f = -c*Iw/R - c*Iw*L/R - c*I/(2*L) - c*m*R/2

        # Tr = T1 - T2 - f
        # Tl = T1 + T2 - f

        # Quintic
        Tl = (-((c*Iw)/R) + (c*Iw*L)/R + (c*It*R)/(2.*L) - (c*m*R)/2. + (120*dist*Iw*t**3)/(R*tf**5) - 
     -  (120*ang*Iw*L*t**3)/(R*tf**5) - (60*ang*It*R*t**3)/(L*tf**5) + (60*dist*m*R*t**3)/tf**5 - 
     -  (180*dist*Iw*t**2)/(R*tf**4) + (180*ang*Iw*L*t**2)/(R*tf**4) + (90*ang*It*R*t**2)/(L*tf**4) - 
     -  (90*dist*m*R*t**2)/tf**4 + (60*dist*Iw*t)/(R*tf**3) - (60*ang*Iw*L*t)/(R*tf**3) - (30*ang*It*R*t)/(L*tf**3) + 
     -  (30*dist*m*R*t)/tf**3)

        Tr = (-((c*Iw)/R) - (c*Iw*L)/R - (c*It*R)/(2.*L) - (c*m*R)/2. + (120*dist*Iw*t**3)/(R*tf**5) + 
     -  (120*ang*Iw*L*t**3)/(R*tf**5) + (60*ang*It*R*t**3)/(L*tf**5) + (60*dist*m*R*t**3)/tf**5 - 
     -  (180*dist*Iw*t**2)/(R*tf**4) - (180*ang*Iw*L*t**2)/(R*tf**4) - (90*ang*It*R*t**2)/(L*tf**4) - 
     -  (90*dist*m*R*t**2)/tf**4 + (60*dist*Iw*t)/(R*tf**3) + (60*ang*Iw*L*t)/(R*tf**3) + (30*ang*It*R*t)/(L*tf**3) + 
     -  (30*dist*m*R*t)/tf**3)
        return Tr, Tl

    def objective_update(self, obj_pos, *args):
        self.objective = obj_pos
        self.move_finished = True

    def send_draw_data(self, *args):
        stdout.flush()
        bodies = list(self.verts.values())
        if self.projectile:
            projectile = {'POS': self.projectile.pos,
                          'SIZE': self.projectile.size,
                          'COLOR': self.projectile.color}
        else:
            projectile = None
        objective = self.objective
        data = {'STATE': {
                         'POSITION': self.pos,
                         'VELOCITY': self.vel,
                         'TORQUE': self.torque,
                         'BASIS': self.basis,
                         'IS_AUTO': self.AUTO},
                'BODIES': bodies,
                'PROJECTILE': projectile,
                'OBJECTIVE': objective,
                'T': self.time}
        self.pg_comm_pipe.send({'GFX_DATA': data})

    def parse_signal(self):
        has_received = self.pg_comm_pipe.poll()
        if has_received:
            signal = self.pg_comm_pipe.recv()
            key, vals = list(signal.keys())[0], list(signal.values())[0]
            # if key != "PLS_SEND":
            #     print("Robot received signal %s with vargs %s" % (key, str(vals)))
            #     stdout.flush()
            self.mp_dict[key](vals)


def Rot2D(vector, theta):
    c = np.cos(theta)
    s = np.sin(theta)
    m = np.matrix([[c, -s], [s, c]])
    return (np.dot(m, vector)).A[0]


def Centroid(poly_points):
    return np.array([
        np.mean(poly_points[:, 0], dtype=np.float32),
        np.mean(poly_points[:, 1], dtype=np.float32)
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