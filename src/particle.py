
class Particle():

    def __init__(self, window, pos, size, vel=[0, 0], color=(255, 255, 255)):
        self.window = window
        self.pos = pos
        self.size = size
        self.color = color
        self.vel = vel

    def update(self, dt):
        self.pos += self.vel*dt
        w_x, w_y = self.window.map_(*self.pos)
        if w_x > self.window.w or w_x < 0 or w_y > self.window.h or w_y < 0:
            return False
        return True
