from math import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Flywheel:
    def __init__(self, radius, mass, rpm, angle, f_c):
        self.radius = radius
        self.mass = mass
        self.rad_s = rpm*2*pi/60
        self.ball_mass = 0.165*0.075
        self.angle = angle
        self.friction_coefficient = f_c        

    def calculate_tangential_velocity(self, rad_s, radius):
        return rad_s * radius
    
    def calculate_angular_momentum(self):
        return 0.5 * self.mass * self.radius**2 * self.rad_s
    
    def ball_exit_velocity(self):
        initial_angular_momentum = self.calculate_angular_momentum()
        moment_of_inertia = 0.5 * self.mass * self.radius**2
        new_angular_velocity = initial_angular_momentum / (moment_of_inertia + self.ball_mass * self.radius**2)
        return new_angular_velocity * self.radius
    
    def findall(self):
        ball_velocity = self.ball_exit_velocity()
        launch_distance = 0.05
        gravity_component = 9.81 * sin(radians(self.angle))
        friction_component = self.friction_coefficient * 9.81 * cos(radians(self.angle))
        net_accel = ball_velocity**2 / (2 * launch_distance) - gravity_component - friction_component
        time = sqrt(2 * launch_distance / net_accel)
        l_velocity = ball_velocity
        x = 0.0
        y = 0.0
        x_vals = [x]
        y_vals = [y]
        
        d = 0.127               # m
        A = pi * (d/2)**2  # cross-sectional area
        rho = 1.225             # kg/m^3
        Cd = 0.5
        g = 9.81
        dt = 0.01
        v_x = l_velocity * cos(radians(self.angle))
        v_y = l_velocity * sin(radians(self.angle)) 
        while y >= 0:
            v = sqrt(v_x**2 + v_y**2)
            
            # Drag accelerations
            a_x = -(0.5 * rho * Cd * A * v * v_x) / self.ball_mass
            a_y = -g - (0.5 * rho * Cd * A * v * v_y) / self.ball_mass
            
            # Update velocities
            v_x += a_x * dt
            v_y += a_y * dt
            
            # Update positions
            x += v_x * dt
            y += v_y * dt
            
            # Store trajectory
            x_vals.append(x)
            y_vals.append(y)

        return x_vals, y_vals

if __name__ == "__main__":
    flywheel = Flywheel(0.127, 0.07, 6000, 25, 0.01)
    x_vals, y_vals = flywheel.findall()
    print(x_vals, y_vals)
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'bo-', markersize=3)
    ax.set_xlim(0, max(x_vals) + 1)
    ax.set_ylim(0, max(y_vals) + 1)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.grid(True)
    
    def animate(frame):
        line.set_data(x_vals[:frame+1], y_vals[:frame+1])
        return line,
    
    ani = animation.FuncAnimation(fig, animate, frames=len(x_vals), interval=50, blit=True)
    plt.show()