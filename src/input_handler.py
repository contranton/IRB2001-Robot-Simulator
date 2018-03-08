import numpy as np
import pygame as pg
from sys import exit


class Handler():

    def __init__(self, window, robot_comm_pipe):
        self.window = window
        self.robot_comm_pipe = robot_comm_pipe
        # Initialize UI control booleans
        self.M_DOWN, self.M_UP, self.M_MOTION, self.M_LEFT, \
            self.M_RIGHT, self.M_MIDDLE = [0]*6
        self.FBRL = 0b0000

    def _has_wasd_changed(self, FBRL):
        return FBRL != self.FBRL

    def input_handler(self):
        window = self.window

        for event in pg.event.get():

            # Get mouse state
            self.M_DOWN = event.type == pg.MOUSEBUTTONDOWN
            self.M_UP = event.type == pg.MOUSEBUTTONUP
            self.M_MOTION = event.type == pg.MOUSEMOTION
            self.M_LEFT, self.M_MIDDLE, self.M_RIGHT = pg.mouse.get_pressed()

            # Get keys state
            keys = pg.key.get_pressed()
            F = keys[pg.K_UP] * 0b1000
            B = keys[pg.K_DOWN] * 0b0100
            R = keys[pg.K_RIGHT] * 0b0010
            L = keys[pg.K_LEFT] * 0b0001
            FBRL = F | B | R | L

            # Only send wasd signal if key combination has changed
            if self._has_wasd_changed(FBRL):
                # Handle movement
                self.robot_comm_pipe.send({"WASD": FBRL})
                self.FBRL = FBRL

            # Scrolling
            if self.M_DOWN:
                # Scroll up
                if event.button == 4:
                    window.scale_screen(-1)
                # Scroll down
                if event.button == 5:
                    window.scale_screen(1)

            # Handle close
            if event.type == pg.QUIT:
                return False

            # Handle special keys (non-movement)
            if event.type == pg.KEYDOWN:

                # Reset robot pose
                if event.key == pg.K_r:
                    # robot.reset()
                    self.robot_comm_pipe.send({"RESET": None})
                    window.reset()

                # Pause system
                if event.key == pg.K_p:
                    paused = True
                    self.paused_loop(paused)

                # Quit through ESC key
                if event.key == pg.K_ESCAPE:
                    return False

                # Robot brake on
                if event.key == pg.K_s:
                    # robot.brake(brake)
                    self.robot_comm_pipe.send({"BRAKE_ON": None})

                # Show torque graphic
                if event.key == pg.K_t:
                    window.WILL_TQ = True

                # Fire projectile on spacebar
                if event.key == pg.K_SPACE:
                    # robot.fire()
                    self.robot_comm_pipe.send({"FIRE": None})

                # Toggle Automatic kinematic mode
                if event.key == pg.K_a:
                    # robot.AUTO = not robot.AUTO
                    self.robot_comm_pipe.send({"TOG_AUTO": None})

            elif event.type == pg.KEYUP:
                # Robot brake off
                if event.key == pg.K_s:
                    # robot.brake(brake)
                    self.robot_comm_pipe.send({"BRAKE_OFF": None})


        # Handle mouse
        if self.M_DOWN:
            if self.M_RIGHT:
                pg.mouse.get_rel()
            if self.M_LEFT:
                self.objective_update()
        if self.M_MOTION:
            if self.M_RIGHT:
                M_mov = np.array(pg.mouse.get_rel())
                window.move_screen(M_mov)
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
        obj_pos = self.window.invmap_(*pg.mouse.get_pos())
        self.robot_comm_pipe.send({"OBJ_UPDATE": obj_pos})

    def check_projectile(self, projectile_pos):
        w_x, w_y = self.window.map_(projectile_pos)
        if w_x > self.window.w or w_x < 0 or w_y > self.window.h or w_y < 0:
            self.robot_comm_pipe.send("PROJ_DEAD")
