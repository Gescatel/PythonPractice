import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class dubinscar:
    def __init__(self, x, y, theta, v):
        # State of car (i.e., x and y position)
        self.x = x
        self.y = y
        self.v = v 
        self.theta = theta
    
    # This function should update self.x, self.y, and self.theta with the new position and orientation
    def step(self, u, dt):
        self.u = u
        self.dt = dt
        self.x = self.x + self.v * np.cos(self.theta) * dt
        self.y = self.y + self.v * np.sin(self.theta) * dt
        self.theta = self.theta + self.v * np.tan(self.u) * dt

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.error_sum = 0.0
        self.error_prev = 0.0
    
    def control(self, error, dt):
        d_error = (error - self.error_prev) / dt
        self.error_sum += error * dt
        self.error_prev = error
        return self.Kp * error + self.Ki * self.error_sum + self.Kd * d_error
        
# Simulate the dubins car for some time
def simulate(car, controller, track_function, total_time, dt):
    ts = np.arange(0, total_time, dt)
    xs, ys, thetas = [], [], []
    
    for t in ts:
        _, target_y = track_function(t)
        error = target_y - car.y
        u = controller.control(error, dt)
        car.step(u, dt)
        xs.append(car.x)
        ys.append(car.y)
        thetas.append(car.theta)
        
    return ts, xs, ys, thetas

def track_function(t):
    return t, 0

###### Simulation Parameters ######
# Initial conditions and instantitate car
car = dubinscar(0.0, 1.0, 5., 0.5)
total_time = 10.0
dt = 0.01
# Set PID Gains here
Kp = 1.0
Ki = 0.0
Kd = 0.0

controller = PIDController(Kp, Ki, Kd)

ts, xs, ys, thetas = simulate(car, controller, track_function, total_time, dt)

# Prepare figure
fig, ax = plt.subplots()
ax.set_xlim((0, total_time))
ax.set_ylim((-2, 2))
car_dot, = plt.plot([], [], 'bo') # car position dot
track, = plt.plot([], [], 'ro') # track

def animate(i):
    car_dot.set_data(xs[i], ys[i])
    track.set_data(ts[:i+1], [0]*len(ts[:i+1]))
    return car_dot, track

anim = FuncAnimation(fig, animate, frames=len(ts), interval=dt*1000, blit=True)
plt.show()