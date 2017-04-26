import numpy as np
import pygame as pg
from sys import exit

class Handler():

    def __init__(self, WIN, robot):
        self.WIN, self.robot = WIN, robot
        # Initialize UI control booleans
        self.M_DOWN, self.M_UP, self.M_MOTION, self.M_LEFT, \
            self.M_RIGHT, self.M_MIDDLE = [0]*6

    def input_handler(self):
        WIN, robot = self.WIN, self.robot

        for event in pg.event.get():

            # Get mouse state
            self.M_DOWN = event.type == pg.MOUSEBUTTONDOWN
            self.M_UP = event.type == pg.MOUSEBUTTONUP
            self.M_MOTION = event.type == pg.MOUSEMOTION
            self.M_LEFT, self.M_MIDDLE, self.M_RIGHT = pg.mouse.get_pressed()

            # Scrolling
            if self.M_DOWN:
                # Scroll up
                if event.button == 4:
                    WIN.scale_screen(-1)
                # Scroll down
                if event.button == 5:
                    WIN.scale_screen(1)

            # Handle close
            if event.type == pg.QUIT:
                return False

            # Handle special keys (non-movement)
            if event.type == pg.KEYDOWN:

                # Reset robot pose
                if event.key == pg.K_r:
                    robot.reset()
                    WIN.reset()

                # Pause system
                if event.key == pg.K_p:
                    paused = True
                    self.paused_loop(paused)

                # Quit through ESC key
                if event.key == pg.K_ESCAPE:
                    return False

                # Show torque graphic
                if event.key == pg.K_t:
                    WIN.WILL_TQ = True

                # Fire projectile on spacebar
                if event.key == pg.K_SPACE:
                    robot.fire()

                # Toggle Automatic kinematic mode
                if event.key == pg.K_a:
                    robot.AUTO = not robot.AUTO

        keys = pg.key.get_pressed()

        # Handle movement
        if not robot.AUTO:
            F = keys[pg.K_UP]
            L = keys[pg.K_LEFT]
            R = keys[pg.K_RIGHT]
            B = keys[pg.K_DOWN]
            robot.wasd_manipulate(F, B, R, L)

        # Brake despite being on auto
        brake = keys[pg.K_s]
        robot.brake(brake)

        # Handle mouse
        if self.M_DOWN:
            if self.M_RIGHT:
                pg.mouse.get_rel()
            if self.M_LEFT:
                self.objective_update()
        if self.M_MOTION:
            if self.M_RIGHT:
                M_mov = np.array(pg.mouse.get_rel())
                WIN.move_screen(M_mov)
            if self.M_LEFT:
                self.objective_update()

        return True

    def paused_loop(self, val):
        paused = val
        while paused:
            for event in pg.event.get():
                if event.type == pg.KEYDOWN and event.key == pg.K_p:
                    paused = False
                if event.type == pg.QUIT:
                    pg.quit()
                    exit()

    def objective_update(self):
        self.robot.objective = self.WIN.invmap_(*pg.mouse.get_pos())
        self.robot.move_finished = True
        # print("OBJECTIVE:", self.robot.objective)
        # print(robot.goal_angle(objective), robot.goal_distance(objective))

