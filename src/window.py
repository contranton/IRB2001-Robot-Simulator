from math import modf
import pygame as pg


class Window():

    def __init__(self, w=800, h=600, x0=0, y0=0, scale=20):
        self.calc_vars(w, h, x0, y0, scale)
        self.min_scale = 5
        self.max_scale = 100
        self.initial_scale = scale
        self.initial_dims = (w, h)

        # Var used to show torque graph at the end of run
        self.WILL_TQ = False

    def calc_vars(self, w, h, x0, y0, scale):
        # Main Window size
        # w, h
        ar = w / h

        # Vector space bounds
        # Defined with arbitrary a_ and b_ that set the visible width, and arbitrary y_
        # placed in the center of the screen shuch that there is a square
        # aspect ratio.
        a_, b_ = x0-scale/2, x0+scale/2
        y_ = y0
        c_ = y_ - abs(a_-b_)/(2*ar)
        d_ = y_ + abs(a_-b_)/(2*ar)
        # Scaling factor from vector to window space
        alpha_ = w/abs(b_-a_)
        beta_ = h/abs(d_-c_)
        # Center point in vector space
        ox_ = (a_+b_)/2
        oy_ = (c_+d_)/2
        # Number of whole units per direction
        nx_ = int(abs(b_ - a_))
        ny_ = int(abs(d_ - c_))

        # Set all variables as instance variables
        self.w, self.h = w, h
        self.x0, self.y0 = x0, y0
        self.a_, self.b_ = a_, b_
        self.c_, self.d_ = c_, d_
        self.alpha_, self.beta_ = alpha_, beta_
        self.ox_, self.oy_ = ox_, oy_
        self.nx_, self.ny_ = nx_, ny_
        self.scale = scale

    def map_(self, x, y):
        # Vector space to Window space
        i = (x-self.a_)*self.alpha_
        j = -(y-self.d_)*self.beta_
        return (int(i), int(j))

    def mapall_(self, points):
        return [self.map_(x, y) for x, y in points]

    def invmap_(self, i, j):
        # Window space to approx. Vector Space
        x = i/self.alpha_ + self.a_
        y = -j/self.beta_ + self.d_
        return (x, y)

    def coordinate_plane(self, surf):
        map_ = self.map_
        a_, b_, c_, d_ = self.a_, self.b_, self.c_, self.d_

        # Unit grid
        for i in range(0, self.nx_+1):
            a0 = modf(a_)[1]
            pg.draw.aaline(surf, (70, 70, 70),
                           map_(a0+i, d_), map_(a0+i, c_))
        for j in range(0, self.ny_+1):
            d0 = modf(d_)[1]
            pg.draw.aaline(surf, (70, 70, 70),
                           map_(a_, d0-j), map_(b_, d0-j))

        # Main axes
        pg.draw.aaline(surf, (255, 255, 255), map_(0, d_), map_(0, c_))
        pg.draw.aaline(surf, (255, 255, 255), map_(a_, 0), map_(b_, 0))

        # Origin point
        pg.draw.circle(surf, (255, 0, 0), map_(0, 0), 5)

    def move_screen(self, vector):
        x, y = vector*self.scale/1000
        self.calc_vars(
            self.w, self.h,
            self.x0 - x, self.y0 + y,
            self.scale)

    def scale_screen(self, direction):
        new_scale = self.scale + self.scale*direction/10
        if self.min_scale < new_scale < self.max_scale:
            self.calc_vars(
                self.w, self.h,
                self.x0, self.y0,
                new_scale)

    def reset(self):
        self.calc_vars(*self.initial_dims, 0, 0, self.initial_scale)
