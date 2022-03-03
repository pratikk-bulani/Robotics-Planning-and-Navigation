class Obstacle:
    def __init__(self, x, y, R):
        self.x = x
        self.y = y
        self.R = R


OBSTACLES = [
    Obstacle(140, 120, 100),
    Obstacle(420, 390, 100),
    Obstacle(200, 330, 100),
]


n = 30 # Number of way points

x0, y0 = (20.0, 20.0) # Initial point

xg, yg = (450.0, 500.0) # Goal

vmax = 9 # Max velocity

dt = 3 # Time interval

mpciters = 20 # Iterations of MPC

debug = False
