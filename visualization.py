from ursina import *
import pandas as pd
import math
import os
from panda3d.core import Quat, Vec3
from ursina.prefabs.first_person_controller import FirstPersonController

class WindowManager:
    def __init__(self):
        window.title = 'Drone Trajectory Visualization'
        window.borderless = False
        window.exit_button.visible = True
        window.fps_counter.enabled = True
        window.color = color.rgb(0.02, 0.03, 0.1)

class Drone(Entity):
    def __init__(self, trajectory_data, **kwargs):
        super().__init__(**kwargs)
        self.trajectory_data = trajectory_data
        self.current_index = 0
        self.trail = []
        self.trail_length = 20
        self.time_accumulator = 0
        self.last_time = 0
        self.trail_counter = 0
        
        # Drone main body
        self.body = Entity(
            parent=self,
            model='cube',
            color=color.orange,
            scale=(1, 0.2, 1),
            texture='white_cube'
        )
        
        # Coordinate system axes
        self.setup_axes()
        
        # Propellers (4 small cubes)
        self.propellers = [
            Entity(parent=self, model='cube', color=color.white, scale=(0.2, 0.05, 0.2), position=(0.5, 0, 0.5)),
            Entity(parent=self, model='cube', color=color.white, scale=(0.2, 0.05, 0.2), position=(-0.5, 0, 0.5)),
            Entity(parent=self, model='cube', color=color.white, scale=(0.2, 0.05, 0.2), position=(0.5, 0, -0.5)),
            Entity(parent=self, model='cube', color=color.white, scale=(0.2, 0.05, 0.2), position=(-0.5, 0, -0.5))
        ]

    def setup_axes(self):
        """Create colored axes to show drone's orientation"""
        # X-axis (Red)
        self.x_axis = Entity(
            parent=self,
            model='arrow',
            color=color.red,
            scale=(0.5, 0.1, 0.1),
            rotation=(0, 0, 90)
        )
        # Y-axis (Green)
        self.y_axis = Entity(
            parent=self,
            model='arrow',
            color=color.green,
            scale=(0.5, 0.1, 0.1),
            rotation=(90, 0, 0)
        )
        # Z-axis (Blue)
        self.z_axis = Entity(
            parent=self,
            model='arrow',
            color=color.blue,
            scale=(0.5, 0.1, 0.1),
            rotation=(0, 90, 0)
        )

    def update(self):
        """Update drone position based on time from CSV"""
        if self.current_index >= len(self.trajectory_data):
            return

        current_time = self.trajectory_data.iloc[self.current_index]['time']
        if self.last_time == 0:
            self.last_time = current_time

        self.time_accumulator += time.dt
        if self.time_accumulator >= (current_time - self.last_time):
            self.time_accumulator = 0
            self.last_time = current_time
            self.update_position()
            self.current_index += 1
        print(self.rotation)

    '''
    def update_position(self):
        """Update drone's position and orientation from CSV row"""
        row = self.trajectory_data.iloc[self.current_index]
        
        # Position (convert to Ursina coordinate system: X-right, Y-up, Z-forward)
        self.position = (row['x'], row['z'], row['y'])
        
        # Orientation (convert from radians to degrees)
        roll = math.degrees(row['phi'])
        pitch = math.degrees(row['theta'])
        yaw = math.degrees(row['psi'])
        
        # Create quaternion for accurate 3D rotation
        quat = Quat()
        #print(quat)
        quat.setHpr(Vec3(yaw, pitch, roll))
        self.rotation = quat*180
        
        self.update_trail()
    '''
    def update_position(self):
        """Update drone's position and orientation from CSV row"""
        row = self.trajectory_data.iloc[self.current_index]
        
        # Position (convert to Ursina coordinate system: X-right, Y-up, Z-forward)
        self.position = (row['x'], row['z'], row['y'])
        
        # Get rotation angles from CSV (these are in radians)
        roll_rad = row['phi']     # Rotation around X axis (bank)
        pitch_rad = row['theta']  # Rotation around Y axis (attitude)
        yaw_rad = row['psi']      # Rotation around Z axis (heading)
        
        """
        Rotation Pipeline Explanation:
        1. CSV contains angles in radians
        2. Panda3D's setHpr() expects degrees in Vec3
        3. Ursina's rotation property expects Euler angles in degrees
        
        We need to:
        - Convert the CSV radians to degrees for Ursina
        - Apply proper rotation order (usually yaw-pitch-roll)
        """
        
        # OPTION 1: Direct Euler angles (degrees) - simpler but might have gimbal lock
        '''
        self.rotation = (
            math.degrees(pitch_rad),  # X rotation (pitch) in degrees
            math.degrees(yaw_rad),    # Y rotation (yaw) in degrees
            math.degrees(roll_rad)     # Z rotation (roll) in degrees
        )
        '''
        
        # OPTION 2: Quaternion-based rotation (more accurate but more complex)
        quat = Quat()
        # Convert to degrees for setHpr (Panda3D expects degrees here)
        quat.setHpr(Vec3(
            math.degrees(-yaw_rad),   # Heading (Z)
            math.degrees(-pitch_rad), # Pitch (Y)
            math.degrees(-roll_rad)    # Roll (X)
        ))
        # Convert quaternion back to Euler angles for Ursina
        hpr = quat.getHpr()
        self.rotation = (hpr[1], hpr[0], hpr[2])  # Reorder to X,Y,Z
        
        self.update_trail()

    def update_trail(self):
        """Create a visual trail of the drone's path"""
        if self.trail_counter % self.trail_length == 0:
            trail_point = Entity(
                model='sphere',
                color=color.cyan,
                scale=0.1,
                position=self.position
            )
        
            self.trail.append(trail_point)
        self.trail_counter += 1

        #if len(self.trail) > self.trail_length:
        #    destroy(self.trail.pop(0))

class DataLoader:
    def __init__(self, filepath):
        """Load trajectory data from CSV file"""
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"CSV file not found: {filepath}")
        
        self.data = pd.read_csv(filepath)
        
        # Validate required columns
        required_columns = ['time', 'x', 'y', 'z', 'phi', 'theta', 'psi']
        for col in required_columns:
            if col not in self.data.columns:
                raise ValueError(f"CSV missing required column: {col}")

    def get_trajectory_data(self):
        return self.data

class SceneSetup:
    def __init__(self):
        # Setup lighting
        DirectionalLight(position=(0, 10, 0), rotation=(45, -45, 45))
        AmbientLight(color=color.rgba(0.4, 0.4, 0.45, 1))
        
        # Create ground plane
        Entity(
            model='plane',
            texture='grass',
            scale=100,
            rotation=(90, 0, 0),
            collider='mesh'
        )
        
        # Setup camera controller
        self.player = FirstPersonController(
            position=(0, 10, -20),
            rotation=(0, 0, 0),
            speed=10,
            gravity=0
        )
        
        # Add skybox
        Sky()

    def input_handler(self, key):
        """Handle keyboard input"""
        if key == 'q':
            application.quit()
        elif key == 'r':  # Reset camera
            self.player.position = (0, 10, -20)
            self.player.rotation = (0, 0, 0)

def main():
    app = Ursina()
    window_manager = WindowManager()
    
    # Load trajectory data
    try:
        data_loader = DataLoader('drone_simulation_full.csv')
        trajectory_data = data_loader.get_trajectory_data()
    except Exception as e:
        print(f"Error loading data: {e}")
        sys.exit(1)
    
    # Setup scene
    scene_setup = SceneSetup()
    
    # Create drone
    drone = Drone(trajectory_data)
    
    def update():
        drone.update()
        # Camera height control
        scene_setup.player.y += (held_keys['space'] - held_keys['shift']) * time.dt * 10

    def input(key):
        scene_setup.input_handler(key)

    app.run()

if __name__ == '__main__':
    app = Ursina()
    window_manager = WindowManager()
    
    # Load trajectory data
    try:
        data_loader = DataLoader('drone_simulation_full.csv')
        trajectory_data = data_loader.get_trajectory_data()
    except Exception as e:
        print(f"Error loading data: {e}")
        sys.exit(1)
    
    # Setup scene
    scene_setup = SceneSetup()
    
    # Create drone
    drone = Drone(trajectory_data)
    
    def update():
        drone.update()
        # Camera height control
        scene_setup.player.y += (held_keys['space'] - held_keys['shift']) * time.dt * 10

    def input(key):
        scene_setup.input_handler(key)

    app.run()