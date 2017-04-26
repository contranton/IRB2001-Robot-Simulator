# -*- coding: utf-8 -*-
# Javier Contreras    IRB2001-1    2017-1
"""
Created on Tue Mar 28 16:47:49 2017

@author: Javier
"""

import pygame as pg
import numpy as np
import logging

from multiprocessing import Process, Pipe, log_to_stderr
from numpy.linalg import norm
from matplotlib import pyplot as plt
from sys import exit

from robot import Robot
from window import Window
from input_handler import Handler


framerate = 60 # En desuso hasta implementacion de multiprocessing
RUNNING = True

# def sim_loop(robot, handler, out_pipe):

#     print("SIMULATION THREAD", end="\n")
#     message_sent = False

#     while RUNNING:

#         # Update robot
#         robot.update()
#         out_pipe.send([])

#         # Reset user forces
#         robot.torque = [0, 0]


#     print("SIMULATION DONE")

def draw(robot, surface, window, text_renderer, fps):
    # Background
    pg.draw.rect(surface, (0, 0, 0),  (0, 0, window.w, window.h))
    window.coordinate_plane(surface)

    # Draw robot
    robot.draw(surface)
    pg.draw.circle(surface, (0, 0, 255), window.map_(*robot.center), 5)

    # Robot Basis vectors
    for vec in robot.basis:
        pg.draw.line(surface, (255, 255, 0),
                     window.map_(*robot.pos), window.map_(*(vec+robot.pos)))

    # Render text

    #Text on top left
    fps_text = text_renderer.render("FPS: %f" % fps, 1, (255, 0, 0))
    x_text = text_renderer.render("X: %f" % robot.pos[0], 1, (160, 160, 160))
    y_text = text_renderer.render("Y: %f" % robot.pos[1], 1, (160, 160, 160))
    left_text = [fps_text, x_text, y_text]

    # Text on top right
    tq_left_text = text_renderer.render("Tq_L: %f" % robot.torque[
        0], 1, (160, 160, 160))
    tq_right_text = text_renderer.render("Tq_R: %f" % robot.torque[
        1], 1, (160, 160, 160))
    vel_norm_text = text_renderer.render("|v|: %f" % norm(robot.vel),
                                         1, (160, 160, 160))
    is_auto_text = text_renderer.render({0: 'Control manual',
                                         1: 'Automatico'}[robot.AUTO],
                                         1, (160, 160, 160))

    right_text = [tq_left_text, tq_right_text, vel_norm_text, is_auto_text]

    # Text along bottom
    will_show_torque = text_renderer.render(
                {0: '',  1: 'Se mostrara torque al finalizar'}[window.WILL_TQ],
                1, (160, 160, 160))

    for i, text_ in enumerate(left_text):
        surface.blit(text_, (0, 30*i))
    for i, text_ in enumerate(right_text):
        surface.blit(text_, (window.w*4/7, 30*i))
    surface.blit(will_show_torque, (0, window.h*9/10))


    # All done!
    pg.display.flip()


def main():

    # Debug logger
    # mpl = log_to_stderr()
    # mpl.setLevel(logging.INFO)

    # Define window space
    WIN = Window(w=500, h=500, scale=50)

    # Robot
    robot = Robot(width=2, wheelRadius=1, window=WIN, dt=0.001)

    # Control Handler
    handler = Handler(WIN, robot)

    # Pipe to signal initialization of graphical system
    # out_pipe, in_pipe = Pipe()

    # Initialize pygame
    pg.init()

    # Define surfaces
    DRAW_SURF = pg.display.set_mode((WIN.w, WIN.h))
    # im.convert_alpha(DRAW_SURF)
    pg.display.set_caption("IRB2001 - 2017.1")

    # Clock object for fps tracking
    clock = pg.time.Clock()

    # Font object for text display
    text = pg.font.SysFont("monospace", int(WIN.w/25))

    # Parallel simulation processesf
    # in_pipe, out_pipe = Pipe()
    # sim_process = Process(target=sim_loop,
    #                       args=(robot, handler, out_pipe))

    # sim_process.start()

    torque_data = np.empty([1, 2])

    while True:
        clock.tick()
        fps = clock.get_fps()

        # Handle keyboard and mouse inputs
        # On return False, close signal has been sent
        RUNNING = handler.input_handler()
        if not RUNNING:
            # in_pipe.send(0)
            break

        robot.update()

        draw(robot, DRAW_SURF, WIN, text, fps)

        # Add data to torque graph and reset user torque
        torque_data = np.append(torque_data, [robot.torque], axis=0)
        robot.torque *= 0

    pg.quit()

    # View torques applied during run
    if WIN.WILL_TQ:
        t = np.linspace(0, 1, np.shape(torque_data)[0])
        # print(torque_data)
        plt.plot(t, torque_data[:, 0], t, torque_data[:, 1])
        plt.show()

    # sim_process.terminate()
    exit()

if __name__ == '__main__':
    main()
