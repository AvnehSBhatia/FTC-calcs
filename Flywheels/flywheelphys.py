import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.integrate import solve_ivp

ball_mass = 0.075
ball_diameter = 0.127
air_density = 1.225
drag_coefficient = 0.85
gravity = 9.81
launch_distance = 0.05
time_step = 0.01
ramp_friction_coefficient = 0.09
energy_transfer_efficiency = 0.33
spin_decay = 0.995
rolling_resistance = 0.0015
cross_sectional_area = np.pi * (ball_diameter/2)**2

class flywheel:
    def __init__(self, radius, mass, rpm, angle, friction_coef):
        self.radius = radius
        self.mass = mass
        self.moment_of_inertia = 0.5 * mass * radius**2
        self.angular_velocity = rpm * 2 * np.pi / 60
        self.angle = np.radians(angle)
        self.friction_coefficient = friction_coef
        
    def calculate_contact_time(self):
        avg_velocity = self.angular_velocity * self.radius * 0.7
        return launch_distance / avg_velocity if avg_velocity > 0 else 0.1
        
    def calculate_impulse_transfer(self):
        contact_time = self.calculate_contact_time()
        avg_force = (self.moment_of_inertia * self.angular_velocity * energy_transfer_efficiency) / (self.radius * contact_time)
        impulse = avg_force * contact_time
        return impulse / ball_mass
        
    def ball_launch_velocity(self):
        base_velocity = self.calculate_impulse_transfer()
        ramp_acceleration = gravity * (np.sin(self.angle) - ramp_friction_coefficient * np.cos(self.angle))
        ramp_boost = np.sqrt(max(0, 2 * ramp_acceleration * launch_distance))
        return min(base_velocity + ramp_boost, self.angular_velocity * self.radius * 0.95)
        
    def trajectory_derivative(self, t, state):
        x, y, vx, vy = state
        v = np.sqrt(vx**2 + vy**2)
        
        if v < 0.01:
            return [0, 0, 0, 0]
            
        drag_force = 0.5 * air_density * drag_coefficient * cross_sectional_area * v**2
        drag_acceleration = drag_force / ball_mass
        
        ax = -drag_acceleration * vx/v
        ay = -gravity - drag_acceleration * vy/v
        
        if y <= 0 and vy < 0:
            restitution = 0.7
            vy = -vy * restitution
            vx *= 0.9
            y = max(y, 0)
            
        return [vx, vy, ax, ay]
        
    def simulate(self, target_height=None):
            launch_v = self.ball_launch_velocity()
            vx = launch_v * np.cos(self.angle)
            vy = launch_v * np.sin(self.angle)
            x, y = 0, 0
            
            x_vals, y_vals = [x], [y]
            height_crossings = []
            
            while y >= 0:
                v = max(0.001, np.sqrt(vx**2 + vy**2))
                drag = 0.5 * air_density * drag_coefficient * cross_sectional_area * v**2 / ball_mass
                
                vx -= (drag * vx/v) * time_step
                vy -= (gravity + drag * vy/v) * time_step
                x += vx * time_step
                y += vy * time_step
                
                if target_height is not None:
                    if (y_vals[-1] < target_height <= y) or (y_vals[-1] > target_height >= y):
                        t_frac = (target_height - y_vals[-1]) / (y - y_vals[-1])
                        x_cross = x_vals[-1] + t_frac * (x - x_vals[-1])
                        height_crossings.append(x_cross)
                
                x_vals.append(x)
                y_vals.append(y)
            
            total_distance = x_vals[-1]
            return np.array(x_vals), np.array(y_vals), total_distance, height_crossings

if __name__ == "__main__":
    fw = flywheel(0.127, 0.07, 6000, 25, ramp_friction_coefficient)
    
    target_height = float(input("enter target height in meters: "))
    x_vals, y_vals, total_distance, height_crossings = fw.simulate(target_height)
    
    print(f"total distance: {total_distance:.3f} m")
    
    if height_crossings:
        print(f"ball crosses height {target_height:.3f} m at distances:")
        for i, dist in enumerate(height_crossings):
            print(f"  crossing {i+1}: {dist:.3f} m")
    else:
        print(f"ball does not cross height {target_height:.3f} m")
    
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'bo-', markersize=2)
    ax.set_xlim(0, max(x_vals) + 0.5 if len(x_vals) > 0 else 10)
    ax.set_ylim(0, max(y_vals) + 0.5 if len(y_vals) > 0 else 10)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.grid(True)
    
    def animate(frame):
        if frame < len(x_vals):
            line.set_data(x_vals[:frame+1], y_vals[:frame+1])
        return line,
    
    ani = animation.FuncAnimation(fig, animate, frames=len(x_vals), interval=20, blit=True)
    plt.show()