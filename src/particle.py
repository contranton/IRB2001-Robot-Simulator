
class Particle():

    def __init__(self, pos, size, vel=[0, 0], color=(255, 255, 255)):
        self.pos = pos
        self.size = size
        self.color = color
        self.vel = vel

    def update(self, dt):
        self.pos += self.vel*dt

