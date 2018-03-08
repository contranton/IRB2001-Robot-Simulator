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


framerate = 60
RUNNING = True


def sim_loop(handler, out_pipe):

    print("SIMULATION THREAD", end="\n")

    # Robot
    robot = Robot(out_pipe, width=2, wheelRadius=1, dt=0.0008)
    out_pipe.send("START")

    while RUNNING:

        # Update robot
        robot.update()
        robot.torque *= 0

    print("SIMULATION DONE")


def draw(gfx_data, surface, window, text_renderer, fps):
    #################################
    # Background
    pg.draw.rect(surface, (0, 0, 0),  (0, 0, window.w, window.h))
    window.coordinate_plane(surface)

    #################################
    # Draw robot
    mapall_ = window.mapall_
    map_ = window.map_
    rob_state = gfx_data["STATE"]
    body_parts = gfx_data["BODIES"]
    projectile = gfx_data["PROJECTILE"]
    objective = gfx_data["OBJECTIVE"]

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
        radius = int(window.alpha_*projectile["SIZE"])
        pg.draw.circle(surface, projectile["COLOR"],
                       map_(*projectile["POS"]), radius)

    # Draw objective ball
    if objective is not None:
        radius = int(window.alpha_)
        try:
            pg.gfxdraw.aacircle(surface, map_(*objective),
                                radius, (100, 200, 0))
        except:
            pg.draw.circle(surface, (100, 200, 0),
                           map_(*objective), radius)

    # Robot Basis vectors
    for vec in rob_state["BASIS"]:
        pg.draw.line(surface, (255, 255, 0),
                     window.map_(*rob_state["POSITION"]),
                     window.map_(*(vec + rob_state["POSITION"])))

    ###############################
    # Render text

    #Text on top left
    fps_text = text_renderer.render("FPS: %f" % fps, 1, (255, 0, 0))
    x_text = text_renderer.render("X: %f" % rob_state["POSITION"][0],
                                  1, (160, 160, 160))
    y_text = text_renderer.render("Y: %f" % rob_state["POSITION"][1],
                                  1, (160, 160, 160))
    left_text = [fps_text, x_text, y_text]

    # Text on top right
    tq_left_text = text_renderer.render("Tq_L: %f" % rob_state["TORQUE"][0],
                                        1, (160, 160, 160))
    tq_right_text = text_renderer.render("Tq_R: %f" % rob_state["TORQUE"][1],
                                         1, (160, 160, 160))
    vel_text = text_renderer.render("|v|: %f" % norm(rob_state["VELOCITY"]),
                                    1, (160, 160, 160))
    is_auto_text = text_renderer.render({0: 'Manual',
                                         1: 'Auto'}[rob_state["IS_AUTO"]],
                                         1, (160, 160, 160))

    right_text = [tq_left_text, tq_right_text, vel_text, is_auto_text]

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
    global RUNNING

    ############################
    # Initialize

    # Debug logger
    mpl = log_to_stderr()
    mpl.setLevel(logging.INFO)

    # Pipe for simulation-pygame communication
    pipe1_pg, pipe1_rob = Pipe()

    # Define window space
    WIN = Window(w=500, h=500, scale=50)

    # Control Handler
    handler = Handler(WIN, pipe1_pg)

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

    # Parallel simulation processes
    sim_process = Process(target=sim_loop,
                          args=(handler, pipe1_rob))
    sim_process.start()

    torque_data = np.empty([1, 2])

    # Aux bool for pipe communnication
    signal_sent = False

    ###############################
    # Only start if initial signal has been sent, indicating proper creation
    # of simulation thread
    while True:
        if pipe1_pg.recv() == "START":

            # Main loop
            while RUNNING:
                clock.tick(framerate*2)
                fps = clock.get_fps()

                # Handle keyboard and mouse inputs
                # On return False, close signal has been sent
                RUNNING = handler.input_handler()

                # Ask robot to send gfx data
                if not signal_sent:
                    pipe1_pg.send({"PLS_SEND": None})
                    signal_sent = True

                # Wait to receive gfx from robot process
                if pipe1_pg.poll():
                    gfx_data = pipe1_pg.recv()["GFX_DATA"]
                    signal_sent = False
                    torque_data = np.append(torque_data,
                                            [gfx_data["STATE"]["TORQUE"]],
                                            axis=0)
                    if gfx_data["PROJECTILE"] is not None:
                        handler.check_projectile(gfx_data["PROJECTILE"]["POS"])
                    draw(gfx_data, DRAW_SURF, WIN, text, fps)

            break

    #############################
    # Finalize

    pg.quit()

    # View torques applied during run
    if WIN.WILL_TQ:
        t = np.linspace(0, 1, np.shape(torque_data)[0])
        # print(torque_data)
        plt.plot(t, torque_data[:, 0], t, torque_data[:, 1])
        plt.show()

    sim_process.terminate()
    exit()


    main()
