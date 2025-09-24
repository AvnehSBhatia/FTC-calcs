from math import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Static constants
BALL_MASS = 0.165 * 0.075  # kg
BALL_DIAMETER = 0.127  # m
AIR_DENSITY = 1.225  # kg/m^3
DRAG_COEFFICIENT = 0.5
GRAVITY = 9.81  # m/s^2
LAUNCH_DISTANCE = 0.05  # m
TIME_STEP = 0.01  # s
RAMP_FRICTION_COEFFICIENT = 0.01

class Flywheel:
    def __init__(self, radius, mass, rpm, angle, f_c):
        self.radius = radius
        self.mass = mass
        self.rad_s = rpm*2*pi/60
        self.angle = angle
        self.friction_coefficient = f_c        

    def calculate_tangential_velocity(self, rad_s, radius):
        return rad_s * radius
    
    def calculate_angular_momentum(self):
        return 0.5 * self.mass * self.radius**2 * self.rad_s
    
    def ball_exit_velocity(self):
        initial_angular_momentum = self.calculate_angular_momentum()
        moment_of_inertia = 0.5 * self.mass * self.radius**2
        new_angular_velocity = initial_angular_momentum / (moment_of_inertia + BALL_MASS * self.radius**2)
        return new_angular_velocity * self.radius
    
    def findall(self, target_height=None):
        ball_velocity = self.ball_exit_velocity()
        gravity_component = GRAVITY * sin(radians(self.angle))
        friction_component = self.friction_coefficient * GRAVITY * cos(radians(self.angle))
        net_accel = ball_velocity**2 / (2 * LAUNCH_DISTANCE) - gravity_component - friction_component
        time = sqrt(2 * LAUNCH_DISTANCE / net_accel)
        l_velocity = ball_velocity
        x = 0.0
        y = 0.0
        x_vals = [x]
        y_vals = [y]
        
        A = pi * (BALL_DIAMETER/2)**2  # cross-sectional area
        v_x = l_velocity * cos(radians(self.angle))
        v_y = l_velocity * sin(radians(self.angle))
        
        height_crossings = []
        prev_y = y
        
        while y >= 0:
            v = sqrt(v_x**2 + v_y**2)
            
            # Drag accelerations
            a_x = -(0.5 * AIR_DENSITY * DRAG_COEFFICIENT * A * v * v_x) / BALL_MASS
            a_y = -GRAVITY - (0.5 * AIR_DENSITY * DRAG_COEFFICIENT * A * v * v_y) / BALL_MASS
            
            # Update velocities
            v_x += a_x * TIME_STEP
            v_y += a_y * TIME_STEP
            
            # Update positions
            x += v_x * TIME_STEP
            y += v_y * TIME_STEP
            
            # Check for height crossing
            if target_height is not None:
                if (prev_y < target_height <= y) or (prev_y > target_height >= y):
                    height_crossings.append(x)
            
            # Store trajectory
            x_vals.append(x)
            y_vals.append(y)
            prev_y = y

        total_distance = x_vals[-1]
        return x_vals, y_vals, total_distance, height_crossings

if __name__ == "__main__":
    flywheel = Flywheel(0.127, 0.07, 6000, 25, RAMP_FRICTION_COEFFICIENT)
    
    target_height = float(input("Enter target height in meters: "))
    x_vals, y_vals, total_distance, height_crossings = flywheel.findall(target_height)
    
    print(f"Total distance: {total_distance:.3f} m")
    
    if height_crossings:
        print(f"Ball crosses height {target_height:.3f} m at distances:")
        for i, distance in enumerate(height_crossings):
            print(f"  Crossing {i+1}: {distance:.3f} m")
    else:
        print(f"Ball does not cross height {target_height:.3f} m")
    
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