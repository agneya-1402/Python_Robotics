import pybullet as p
import pybullet_data
import time
import numpy as np
import random

class SelfDrivingCar:
    def __init__(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # grnf
        p.loadURDF("plane.urdf")
        
        # Car
        self.car_id = self.create_car()
        
        # Obstacles
        self.obstacles = self.create_obstacles(num_obstacles=30)
        
        # Ray parameters
        self.num_rays = 8
        self.ray_length = 5
        self.ray_angles = np.linspace(-np.pi/3, np.pi/3, self.num_rays)
        
        # Control parameters
        self.max_velocity = 100
        self.turn_speed = 0.5
        
    def create_car(self):
        base_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[0.3, 0.15, 0.1]
        )
        base_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.3, 0.15, 0.1],
            rgbaColor=[0, 0.5, 1, 1]  # blue color
        )
        
        car = p.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=base_collision,
            baseVisualShapeIndex=base_visual,
            basePosition=[0, 0, 0.2],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        return car
    
    def create_obstacles(self, num_obstacles):
        obstacles = []
        
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2])
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2], rgbaColor=[1, 0, 0, 1]) # red color
        
        for _ in range(num_obstacles):
            x = random.uniform(-5, 5)
            y = random.uniform(-5, 5)
            
            obstacle = p.createMultiBody(
                baseMass=0,  # Static obstacles
                baseCollisionShapeIndex=collision_shape,
                baseVisualShapeIndex=visual_shape,
                basePosition=[x, y, 0.2]
            )
            obstacles.append(obstacle)
        
        return obstacles
    
    def get_ray_sensors(self):
        car_pos, car_orn = p.getBasePositionAndOrientation(self.car_id)
        car_orn = p.getEulerFromQuaternion(car_orn)[2]  # Get yaw angle
        
        distances = []
        
        for angle in self.ray_angles:
            # Calculate ray end point
            ray_angle = car_orn + angle
            ray_end = [
                car_pos[0] + self.ray_length * np.cos(ray_angle),
                car_pos[1] + self.ray_length * np.sin(ray_angle),
                car_pos[2]
            ]
            
            # Raycasting
            result = p.rayTest(car_pos, ray_end)[0]
            hit_fraction = result[2]
            
            # Calculate dist
            distance = hit_fraction * self.ray_length if hit_fraction < 1.0 else self.ray_length
            distances.append(distance)
            
            # show rays 
            hit_pos = [
                car_pos[0] + distance * np.cos(ray_angle),
                car_pos[1] + distance * np.sin(ray_angle),
                car_pos[2]
            ]
            p.addUserDebugLine(car_pos, hit_pos, [1, 1, 0], 1, 0.1)
        
        return distances
    
    def avoid_obstacles(self, sensor_readings):

        # Obstacle avoidance 
        left_distances = sensor_readings[:len(sensor_readings)//2]
        right_distances = sensor_readings[len(sensor_readings)//2:]
        
        # Avg Dist
        left_avg = sum(left_distances) / len(left_distances)
        right_avg = sum(right_distances) / len(right_distances)
        
        # Steering dir
        steering = (right_avg - left_avg) * self.turn_speed
        
        # Frwd Vel
        min_distance = min(sensor_readings)
        velocity = self.max_velocity * (min_distance / self.ray_length)
        
        return velocity, steering
    
    def apply_control(self, velocity, steering):
        # Current position & orientation
        pos, orn = p.getBasePositionAndOrientation(self.car_id)
        orn = list(p.getEulerFromQuaternion(orn))
        
        # Update orientation (yaw angle)
        orn[2] += steering  
        
        # New position
        new_x = pos[0] + velocity * np.cos(orn[2]) * 0.01
        new_y = pos[1] + velocity * np.sin(orn[2]) * 0.01
        
        # Apply new position 
        p.resetBasePositionAndOrientation(
            self.car_id,
            [new_x, new_y, pos[2]],
            p.getQuaternionFromEuler(orn)
        )

def main():
    car = SelfDrivingCar()
    
    while True:
        # Sensor readings
        sensor_readings = car.get_ray_sensors()
        
        # Control cmnds
        velocity, steering = car.avoid_obstacles(sensor_readings)
        car.apply_control(velocity, steering)
        
        # Simulation
        p.stepSimulation()
        time.sleep(1/1240)

if __name__ == "__main__":
    main()