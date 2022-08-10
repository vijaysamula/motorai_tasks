import numpy as np 
import pickle
class LiDAR_object:
    def __init__(self, timestamp, pose_x, pose_y):
        self.timestamp = timestamp
        self.pose_x = pose_x
        self.pose_y = pose_y

class Radar_object:
    def __init__(self, timestamp, pose_x, pose_y, vel_x, vel_y):
        self.timestamp = timestamp
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.vel_x = vel_x
        self.vel_y = vel_y
class Ground_Truth:
    def __init__(self, timestamp, pose_x, pose_y, vel_x, vel_y):
        self.timestamp = timestamp
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.vel_x = vel_x
        self.vel_y = vel_y
    
lidar_file = open("/home/vijay/Documents/CV/cv_2022/motorai/sensor_data_fusion/data/lidar.bin", "rb")
lidar_states = pickle.load(lidar_file)

for i in range(len(lidar_states)):
    pose_x = lidar_states[i].pose_x
    pose_y = lidar_states[i].pose_y
    print(pose_x, pose_y)

radar_file = open("/home/vijay/Documents/CV/cv_2022/motorai/sensor_data_fusion/data/radar.bin", "rb")
radar_states = pickle.load(radar_file)
for j in range(len(radar_states)):
    pose_x = radar_states[j].pose_x
    pose_y = radar_states[j].pose_y
    vel_x = radar_states[j].vel_x
    vel_y = radar_states[j].vel_y
    
ground_truth_file = open("/home/vijay/Documents/CV/cv_2022/motorai/sensor_data_fusion/data/ground_truth.bin", "rb")
ground_states = pickle.load(ground_truth_file)

for k in range(len(ground_states)):
    pose_x = ground_states[k].pose_x
    pose_y = ground_states[k].pose_y
    vel_x = ground_states[k].vel_x
    vel_y = ground_states[k].vel_y

