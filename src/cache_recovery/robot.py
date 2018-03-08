# uncompyle6 version 2.9.10
# Python bytecode 3.6 (3379)
# Decompiled from: Python 3.6.0 |Anaconda 4.3.1 (64-bit)| (default, Dec 23 2016, 11:57:41) [MSC v.1900 64 bit (AMD64)]
# Embedded file name: C:\Users\Javier\Google Drive\UC\2017 - 1\IRB2001\HWs\HW 3\src\robot.py
# Compiled at: 2017-04-26 22:01:59
# Size of source mod 2**32: 12562 bytes
import numpy as np
from particle import Particle
from math import atan2
eps = 1e-31
s = np.sin
c = np.cos

class Robot:
    """
    Object simulating the differential drive robot
    """

    def __init__(self, pg_comm_pipe, width, wheelRadius, mass, pos, ang, vel, omg, torquePulse, dt=(
 1, [0, 0],
 90, 0, 0, 1000, 1)):
        self.dt = dt
        self.pg_comm_pipe = pg_comm_pipe
        self.torque = np.array([0, 0], np.float64, **('dtype', ))
        self.torquePulse = torquePulse
        self.friction_const = 0.1
        self.BRAKE = False
        self.pos = np.array(pos, np.float64, **('dtype', ))
        self.ang = np.radians(ang)
        self.vel = 0
        self.omg = omg
        self.wheelRadius = np.array(wheelRadius, np.float64, **('dtype', ))
        self.width = width
        self.mass = mass
        self.inertia = mass * ((1 + wheelRadius ** 2) / 6 + (width / 2 + 0.5) ** 2 / 12 + ((width ** 2 + (wheelRadius + 1) ** 2) / 12 + 0.25 + width ** 2 + 1 + 4))
        self.wheel_inertia = mass / 5 * self.wheelRadius ** 2
        self.max_torque = torquePulse * 2
        self.verts = {'BODY':[
          np.array([
           [
            -width / 2, -wheelRadius],
           [
            width / 2, -wheelRadius],
           [
            width / 2, wheelRadius + 1],
           [
            -width / 2, wheelRadius + 1]], np.float64, **('dtype', )),
          (255, 100, 100)],'LWHEEL':[
          np.array([
           [
            -width / 2, -wheelRadius],
           [
            -width / 2, wheelRadius],
           [
            -width / 2 - 1, wheelRadius],
           [
            -width / 2 - 1, -wheelRadius]], np.float64, **('dtype', )),
          (0, 150, 255)],'RWHEEL':[
          np.array([
           [
            width / 2, -wheelRadius],
           [
            width / 2, wheelRadius],
           [
            width / 2 + 1, wheelRadius],
           [
            width / 2 + 1, -wheelRadius]], np.float64, **('dtype', )),
          (0, 150, 255)],'PAIL':[
          np.array([
           [
            1.8 * width / 3, wheelRadius + 1],
           [
            2 * width / 3, wheelRadius + 2],
           [
            1.8 * width / 3, wheelRadius + 2],
           [
            1.5 * width / 3, wheelRadius + 1.2],
           [
            -1.5 * width / 3, wheelRadius + 1.2],
           [
            -1.8 * width / 3, wheelRadius + 2],
           [
            -2 * width / 3, wheelRadius + 2],
           [
            -1.8 * width / 3, wheelRadius + 1]]),
          (200, 0, 255)],}
        self.center = Centroid(self.verts['BODY'][0])
        self.transform(Translate, np.negative(self.center), **('vector', ))
        self.basis = np.array([[1, 0], [0, 1]], np.float64, **('dtype', ))
        self.projectile = None
        self.F = 0
        self.B = 0
        self.R = 0
        self.L = 0
        self.AUTO = False
        self.turning = False
        self.moving = False
        self.move_finished = True
        self.objective = None
        self.poly_step = 0
        self.mp_dict = {'WASD':self.update_wasd,'FIRE':self.fire,'TOG_AUTO':self.toggle_auto,'BRAKE':self.brake,'OBJ_UPDATE':self.objective_update,'PROJ_DEAD':self.kill_projectile,'RESET':self.reset,'PLS_SEND':self.send_draw_data,}

    def toggle_auto(self, *args):
        self.AUTO = not self.AUTO

    def kill_projectile(self, *args):
        self.projectile = None

    def update_wasd(self, vals, *args):
        self.F = vals[0]
        self.B = vals[1]
        self.R = vals[2]
        self.L = vals[3]

    def update--- This code section failed: ---

 128       0  LOAD_FAST                'self'
           2  LOAD_ATTR                'parse_signal'
           4  CALL_FUNCTION_0       0 
           6  POP_TOP          

 130       8  LOAD_FAST                'self'
          10  LOAD_ATTR                'dt'
          12  STORE_FAST               'dt'

 131      14  LOAD_FAST                'self'
          16  LOAD_ATTR                'ang'
          18  STORE_FAST               'th'

 132      20  LOAD_FAST                'self'
          22  LOAD_ATTR                'mass'
          24  STORE_FAST               'm'

 133      26  LOAD_FAST                'self'
          28  LOAD_ATTR                'wheelRadius'
          30  STORE_FAST               'R'

 134      32  LOAD_FAST                'self'
          34  LOAD_ATTR                'inertia'
          36  STORE_FAST               'I'

 135      38  LOAD_FAST                'self'
          40  LOAD_ATTR                'wheel_inertia'
          42  STORE_FAST               'Iw'

 136      44  LOAD_FAST                'self'
          46  LOAD_ATTR                'width'
          48  LOAD_CONST            2  2
          50  BINARY_TRUE_DIVIDE
          52  STORE_FAST               'L'

 138      54  LOAD_FAST                'self'
          56  LOAD_ATTR                'AUTO'
          58  POP_JUMP_IF_FALSE    80  'to 80'
          60  LOAD_FAST                'self'
          62  LOAD_ATTR                'objective'
          64  LOAD_CONST               ''
          66  COMPARE_OP               'is not'
        68_0  COME_FROM                '58'
          68  POP_JUMP_IF_FALSE    80  'to 80'

 139      70  LOAD_FAST                'self'
          72  LOAD_ATTR                'auto_control'
          74  CALL_FUNCTION_0       0 
          76  POP_TOP          
          78  JUMP_FORWARD         88  'to 88'
          80  ELSE                     '88'

 141      80  LOAD_FAST                'self'
          82  LOAD_ATTR                'wasd_manipulate'
          84  CALL_FUNCTION_0       0 
          86  POP_TOP          
        88_0  COME_FROM                '78'

 143      88  LOAD_FAST                'self'
          90  LOAD_ATTR                'torque'
          92  UNPACK_SEQUENCE_2     2 
          94  STORE_FAST               'Tr'
          96  STORE_FAST               'Tl'

 146      98  LOAD_FAST                'self'
         100  DUP_TOP          
         102  LOAD_ATTR                'vel'
         104  LOAD_FAST                'Tr'
         106  LOAD_FAST                'Tl'
         108  BINARY_ADD       
         110  LOAD_FAST                'R'
         112  BINARY_TRUE_DIVIDE
         114  LOAD_FAST                'm'
         116  LOAD_CONST            2  2
         118  LOAD_FAST                'Iw'
         120  LOAD_CONST            2  2
         122  BINARY_POWER     
         124  BINARY_MULTIPLY  
         126  LOAD_FAST                'R'
         128  LOAD_CONST            2  2
         130  BINARY_POWER     
         132  BINARY_TRUE_DIVIDE
         134  BINARY_ADD       
         136  BINARY_TRUE_DIVIDE
         138  LOAD_FAST                'dt'
         140  BINARY_MULTIPLY  
         142  INPLACE_ADD      
         144  ROT_TWO          
         146  STORE_ATTR               'vel'

 147     148  LOAD_FAST                'self'
         150  DUP_TOP          
         152  LOAD_ATTR                'omg'
         154  LOAD_FAST                'L'
         156  LOAD_FAST                'Tr'
         158  LOAD_FAST                'Tl'
         160  BINARY_SUBTRACT  
         162  BINARY_MULTIPLY  
         164  LOAD_FAST                'R'
         166  BINARY_TRUE_DIVIDE
         168  LOAD_FAST                'I'
         170  LOAD_CONST            2  2
         172  LOAD_FAST                'Iw'
         174  BINARY_MULTIPLY  
         176  LOAD_FAST                'L'
         178  LOAD_CONST            2  2
         180  BINARY_POWER     
         182  BINARY_MULTIPLY  
         184  LOAD_FAST                'R'
         186  LOAD_CONST            2  2
         188  BINARY_POWER     
         190  BINARY_TRUE_DIVIDE
         192  BINARY_ADD       
         194  BINARY_TRUE_DIVIDE
         196  LOAD_FAST                'dt'
         198  BINARY_MULTIPLY  
         200  INPLACE_ADD      
         202  ROT_TWO          
         204  STORE_ATTR               'omg'

 150     206  LOAD_FAST                'self'
         208  DUP_TOP          
         210  LOAD_ATTR                'vel'
         212  LOAD_CONST            1  1
         214  LOAD_FAST                'self'
         216  LOAD_ATTR                'friction_const'
         218  BINARY_SUBTRACT  
         220  INPLACE_MULTIPLY 
         222  ROT_TWO          
         224  STORE_ATTR               'vel'

 151     226  LOAD_FAST                'self'
         228  DUP_TOP          
         230  LOAD_ATTR                'omg'
         232  LOAD_CONST            1  1
         234  LOAD_FAST                'self'
         236  LOAD_ATTR                'friction_const'
         238  BINARY_SUBTRACT  
         240  INPLACE_MULTIPLY 
         242  ROT_TWO          
         244  STORE_ATTR               'omg'

 154     246  LOAD_FAST                'self'
         248  DUP_TOP          
         250  LOAD_ATTR                'pos'
         252  LOAD_FAST                'self'
         254  LOAD_ATTR                'vel'
         256  LOAD_FAST                'dt'
         258  BINARY_MULTIPLY  
         260  LOAD_GLOBAL              'np'
         262  LOAD_ATTR                'array'
         264  LOAD_GLOBAL              'np'
         266  LOAD_ATTR                'cos'
         268  LOAD_FAST                'th'
         270  CALL_FUNCTION_1       1 
         272  LOAD_GLOBAL              'np'
         274  LOAD_ATTR                'sin'
         276  LOAD_FAST                'th'
         278  CALL_FUNCTION_1       1 
         280  BUILD_LIST_2          2 
         282  CALL_FUNCTION_1       1 
         284  BINARY_MULTIPLY  
         286  INPLACE_ADD      
         288  ROT_TWO          
         290  STORE_ATTR               'pos'

 155     292  LOAD_FAST                'self'
         294  DUP_TOP          
         296  LOAD_ATTR                'ang'
         298  LOAD_FAST                'self'
         300  LOAD_ATTR                'omg'
         302  LOAD_FAST                'dt'
         304  BINARY_MULTIPLY  
         306  INPLACE_ADD      
         308  ROT_TWO          
         310  STORE_ATTR               'ang'

 158     312  LOAD_FAST                'self'
         314  LOAD_ATTR                'transform'
         316  LOAD_GLOBAL              'Translate'
         318  LOAD_FAST                'self'
         320  LOAD_ATTR                'pos'
         322  LOAD_FAST                'self'
         324  LOAD_ATTR                'center'
         326  BINARY_SUBTRACT  
         328  LOAD_CONST               ('vector',)
         330  CALL_FUNCTION_KW_2     2 
         332  POP_TOP          

 159     334  LOAD_FAST                'self'
         336  LOAD_ATTR                'transform'
         338  LOAD_GLOBAL              'Rotate'
         340  LOAD_FAST                'self'
         342  LOAD_ATTR                'omg'
         344  LOAD_FAST                'dt'
         346  BINARY_MULTIPLY  
         348  LOAD_FAST                'self'
         350  LOAD_ATTR                'center'
         352  LOAD_CONST               ('theta', 'axis')
         354  CALL_FUNCTION_KW_3     3 
         356  POP_TOP          

 162     358  LOAD_GLOBAL              'Centroid'
         360  LOAD_FAST                'self'
         362  LOAD_ATTR                'verts'
         364  LOAD_CONST               'BODY'
         366  BINARY_SUBSCR    
         368  LOAD_CONST            0  ''
         370  BINARY_SUBSCR    
         372  CALL_FUNCTION_1       1 
         374  LOAD_FAST                'self'
         376  STORE_ATTR               'center'

 163     378  LOAD_GLOBAL              'Rotate'
         380  LOAD_FAST                'self'
         382  LOAD_ATTR                'basis'
         384  LOAD_FAST                'self'
         386  LOAD_ATTR                'omg'
         388  LOAD_FAST                'dt'
         390  BINARY_MULTIPLY  
         392  CALL_FUNCTION_2       2 
         394  LOAD_FAST                'self'
         396  STORE_ATTR               'basis'

 166     398  LOAD_FAST                'self'
         400  LOAD_ATTR                'projectile'
         402  LOAD_CONST               ''
         404  COMPARE_OP               'is not'
         406  EXTENDED_ARG          1  ''
         408  POP_JUMP_IF_FALSE   242  'to 242'

 169     410  LOAD_FAST                'self'
         412  LOAD_ATTR                'objective'
         414  LOAD_CONST               ''
         416  COMPARE_OP               'is not'
         418  EXTENDED_ARG          1  ''
         420  POP_JUMP_IF_FALSE   222  'to 222'

 170     422  LOAD_FAST                'self'
         424  LOAD_ATTR                'projectile'
         426  LOAD_ATTR                'size'
         428  LOAD_FAST                'self'
         430  LOAD_ATTR                'width'
         432  LOAD_CONST            4  4
         434  BINARY_TRUE_DIVIDE
         436  BINARY_ADD       
         438  STORE_FAST               'r'

 171     440  LOAD_GLOBAL              'np'
         442  LOAD_ATTR                'linalg'
         444  LOAD_ATTR                'norm'
         446  LOAD_FAST                'self'
         448  LOAD_ATTR                'objective'
         450  LOAD_FAST                'self'
         452  LOAD_ATTR                'projectile'
         454  LOAD_ATTR                'pos'
         456  BINARY_SUBTRACT  
         458  CALL_FUNCTION_1       1 
         460  LOAD_FAST                'r'
         462  COMPARE_OP               '<'
         464  EXTENDED_ARG          1  ''
         466  POP_JUMP_IF_FALSE   222  'to 222'

 172     468  LOAD_CONST               ''
         470  LOAD_FAST                'self'
         472  STORE_ATTR               'objective'

 173     474  LOAD_CONST               ''
         476  RETURN_VALUE     

 175     478  LOAD_FAST                'self'
         480  LOAD_ATTR                'projectile'
         482  LOAD_ATTR                'update'
         484  LOAD_FAST                'dt'
         486  CALL_FUNCTION_1       1 
         488  EXTENDED_ARG          1  ''
         490  POP_JUMP_IF_TRUE    242  'to 242'

 176     492  LOAD_CONST               ''
         494  LOAD_FAST                'self'
         496  STORE_ATTR               'projectile'

Parse error at or near `EXTENDED_ARG' instruction at offset 406

    def add_torque(self, Lt, Rt):
        self.torque += np.array([Lt, Rt], np.float64, **('dtype', ))
        self.torque = np.clip(self.torque, -self.max_torque, self.max_torque)

    def transform--- This code section failed: ---

 183       0  SETUP_LOOP           48  'to 48'
           2  LOAD_FAST                'self'
           4  LOAD_ATTR                'verts'
           6  GET_ITER         
           8  FOR_ITER             46  'to 46'
          10  STORE_FAST               'poly'

 184      12  LOAD_FAST                'function'
          14  LOAD_FAST                'self'
          16  LOAD_ATTR                'verts'
          18  LOAD_FAST                'poly'
          20  BINARY_SUBSCR    
          22  LOAD_CONST            0  ''
          24  BINARY_SUBSCR    
          26  BUILD_TUPLE_1         1 
          28  LOAD_FAST                'kwargs'
          30  CALL_FUNCTION_EX      1  ''
          32  LOAD_FAST                'self'
          34  LOAD_ATTR                'verts'
          36  LOAD_FAST                'poly'
          38  BINARY_SUBSCR    
          40  LOAD_CONST            0  ''
          42  STORE_SUBSCR     
          44  JUMP_BACK             8  'to 8'
          46  POP_BLOCK        
        48_0  COME_FROM_LOOP           '0'

Parse error at or near `CALL_FUNCTION_EX' instruction at offset 30

    def reset(self, *args):
        self.transformRotate(-self.ang)self.center('theta', 'axis')
        self.basis = Rotate(self.basis, -self.ang)
        self.pos *= 0
        self.vel *= 0
        self.ang = 0
        self.omg *= 0
        self.AUTO = False
        self.objective = None
        self.projectile = None

    def brake(self, *args):
        self.vel *= 1 - 10 * self.friction_const
        self.torque *= 0

    def fire(self, *args):
        if self.projectile is None:
            proj_pos = self.pos + self.basis[1] * (self.wheelRadius + 1.5)
            proj_vel = 50
            self.projectile = Particle(proj_pos, self.width / 2, np.dot(np.array([0, proj_vel]), self.basis))

    def wasd_manipulate(self):
        F, B, R, L = (
         self.F, self.B, self.R, self.L)
        tq = self.torquePulse
        self.add_torque(tq * ((F - B) * (1 + L * (not R)) + (L - R) * (not (F or B))), tq * ((F - B) * (1 + R * (not L)) + (R - L) * (not (F or B))))

    def goal_angle(self, objective_pos):
        v1 = objective_pos - self.pos
        v2 = self.basis[1]
        norm = np.linalg.norm
        dot = np.dot(v1, v2)
        sense = -np.sign(np.cross(v1, v2))
        angle = np.arccos(dot / (norm(v1) * norm(v2)))
        return (
         angle, sense)

    def goal_distance(self, objective_pos):
        return np.linalg.norm(objective_pos - self.pos)

    def auto_control(self):
        dt = self.dt
        angle, sense = self.goal_angle(self.objective)
        dist = self.goal_distance(self.objective)
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
            tf = 0.2
            tr, tl = self.calc_torque(0, sense * angle, t, tf)
            self.add_torque(tr, tl)
        else:
            if self.moving:
                tf = 0.2
                tr, tl = self.calc_torque(dist, 0, t, tf)
                self.add_torque(tr, tl)
            else:
                self.objective = None
            self.poly_step += 1

    def calc_torque(self, dist, ang, t, tf):
        Iw = self.wheel_inertia
        I = self.inertia
        L = self.width / 2
        m = self.mass
        R = self.wheelRadius
        c = self.friction_const
        T1 = 6 * dist * Iw / R * tf ** 2 + 3 * dist * m * R / tf ** 2 - 12 * dist * Iw * t / (R * tf ** 3)
        T2 = 12 * ang * Iw * L * t / (R * tf ** 3) + 6 * ang * I * R * t / (L * tf ** 3) - 6 * ang * Iw * L / (R * tf ** 2) - 3 * ang * I * R / (L * tf ** 2)
        f = -c * Iw / R - c * Iw * L / R - c * I / (2 * L) - c * m * R / 2
        Tr = T1 - T2 - f
        Tl = T1 + T2 - f
        return (
         -Tr, -Tl)

    def __str__(s):
        s = '\nROBOT ID: %f\nPosition: (%0.2f, %0.2f)\nVelocity: %0.2f\nOrientation: %0.2f\n\nIsAutomatic: %i\nHasFired: %i\n        ' % *(id(s),), *s.pos,
         *(s.vel, s.ang,
          int(s.AUTO), int(s.projectile is not None))
        return s

    def objective_update(self, obj_pos, *args):
        self.objective = obj_pos
        self.move_finished = True

    def send_draw_data(self, *args):
        bodies = list(self.verts.values)
        if self.projectile:
            projectile = {'POS':self.projectile.pos
                          'SIZE':self.projectile.size,
                          'COLOR':self.projectile.color,}
        else:
            projectile = None
        objective = self.objective
        data = {'STATE':{
                         'POSITION':self.pos,
                         'VELOCITY':self.vel,
                         'TORQUE':self.torque,
                         'BASIS':self.basis,
                         'IS_AUTO':self.AUTO},
                'BODIES':bodies,
                'PROJECTILE':projectile,
                'OBJECTIVE':objective,}
        self.pg_comm_pipe.send({'GFX_DATA': data})

    def parse_signal--- This code section failed: ---

 337       0  LOAD_FAST                'self'
           2  LOAD_ATTR                'pg_comm_pipe'
           4  LOAD_ATTR                'poll'
           6  CALL_FUNCTION_0       0 
           8  POP_JUMP_IF_FALSE    88  'to 88'

 338      10  LOAD_FAST                'self'
          12  LOAD_ATTR                'pg_comm_pipe'
          14  LOAD_ATTR                'recv'
          16  CALL_FUNCTION_0       0 
          18  STORE_FAST               'signal'

 339      20  LOAD_GLOBAL              'list'
          22  LOAD_FAST                'signal'
          24  LOAD_ATTR                'keys'
          26  CALL_FUNCTION_0       0 
          28  CALL_FUNCTION_1       1 
          30  LOAD_CONST            0  ''
          32  BINARY_SUBSCR    
          34  LOAD_GLOBAL              'list'
          36  LOAD_FAST                'signal'
          38  LOAD_ATTR                'values'
          40  CALL_FUNCTION_0       0 
          42  CALL_FUNCTION_1       1 
          44  ROT_TWO          
          46  STORE_FAST               'key'
          48  STORE_FAST               'args'

 340      50  LOAD_FAST                'key'
          52  LOAD_CONST               'PLS_SEND'
          54  COMPARE_OP               '!='
          56  POP_JUMP_IF_FALSE    74  'to 74'

 341      58  LOAD_GLOBAL              'print'
          60  LOAD_CONST               'Signal %s received by robot'
          62  LOAD_GLOBAL              'str'
          64  LOAD_FAST                'signal'
          66  CALL_FUNCTION_1       1 
          68  BINARY_MODULO    
          70  CALL_FUNCTION_1       1 
          72  POP_TOP          
        74_0  COME_FROM                '56'

 342      74  LOAD_FAST                'self'
          76  LOAD_ATTR                'mp_dict'
          78  LOAD_FAST                'key'
          80  BINARY_SUBSCR    
          82  LOAD_FAST                'args'
          84  CALL_FUNCTION_EX      0  ''
          86  POP_TOP          
        88_0  COME_FROM                '8'

Parse error at or near `CALL_FUNCTION_EX' instruction at offset 84


def Rot2D(vector, theta):
    c = np.cos(theta)
    s = np.sin(theta)
    m = np.matrix([[c, -s], [s, c]])
    return np.dot(m, vector).A[0]


def Centroid(poly_points):
    return np.array([
     np.mean(poly_points[:, 0], np.float64, **('dtype', )),
     np.mean(poly_points[:, 1], np.float64, **('dtype', ))])


def Translate(poly_points, vector):
    new_points = np.empty_like(poly_points)
    for i in range(len(poly_points)):
        new_points[i] = poly_points[i] + np.array(vector)

    return new_points


def Rotate(poly_points, theta, axis=((0, 0), )):
    new_points = np.empty_like(poly_points)
    for i in range(len(poly_points)):
        new_points[i] = Rot2D(poly_points[i] - axis, theta) + axis

    return new_points


def draw_poly(surf, points, color):
    pg.draw.polygon(surf, color, [map_(i, j) for i, j in points])