import sensor_msgs.point_cloud2 as pc2
from tqdm import tqdm
import numpy as np 

import logging 
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

'''Parses the XYZI data from the Pointcloud2 type of Ros messages'''
def ParseLidarData(lidar_msg_generator, number_of_messages, downsample_rate):
    lidar_data = []
    
    for i in tqdm(range(number_of_messages)):
        _, lidar_msg, lidar_t = next(lidar_messages)
        
        if((i % downsample_rate) != 0):
            continue
        
        xyzi = np.zeros((lidar_msg.height*lidar_msg.width,4))
        laser_idx = 0
        for p in pc2.read_points(lidar_msg, field_names = ("x", "y", "z", "intensity"), skip_nans=True):
            xyzi[laser_idx] = np.array(p[0:4])
        lidar_data.append([lidar_t, xyzi])
        
    logging.info('Parsed the lidar data!')
    return lidar_data

'''Parses the Color Image data from the Image type of Ros messages'''
def ParseColorImageData(image_msg_generator, number_of_messages, downsample_rate):
    color_image_data = []
    
    for i in tqdm(range(number_of_messages)):
        _, image_msg, image_t = next(image_msg_generator)
        
        if((i % downsample_rate) != 0):
            continue
        
        image_data = np.frombuffer(image_msg.data, np.uint8)
        # bgr to rgb 
        image_data_reshaped = np.reshape(image_data, (image_msg.height, image_msg.width, 3))
        image_data_reshaped = image_data_reshaped[:,:,::-1]
        color_image_data.append([image_t, image_data_reshaped])
    logging.info('Parsed the camera data!')
    return color_image_data

'''Parses the Depth Image data from the Image type of Ros messages'''
def ParseDepthImageData(image_msg_generator, number_of_messages, downsample_rate):
    depth_image_data = []
    
    for i in tqdm(range(number_of_messages)):
        _, depth_image_msg, depth_image_t = next(image_msg_generator)
        
        if((i % downsample_rate) != 0):
            continue
        raw_data = np.frombuffer(depth_image_msg.data, np.uint8)
        image_data = np.zeros(depth_image_msg.height * depth_image_msg.width, np.uint16)
        for i in range(depth_image_msg.height * depth_image_msg.width):
            image_data[i] = raw_data[2*i] + raw_data[2*i] << 8
        depth_image_data.append([depth_image_t, image_data])
    
    logging.info('Parsed the camera data!')
    return depth_image_data

'''Parses the VN100 IMU data from the IMU type of Ros messages'''
def ParseVN100ImuData(imu_msg_generator, number_of_messages, downsample_rate):
    imu_data = []
    
    for i in tqdm(range(number_of_messages)):
        _, imu_msg, imu_t = next(imu_msg_generator)
        if((i % downsample_rate) != 0):
            continue
        imu_data.append([imu_t, imu_msg.orientation, imu_msg.orientation_covariance, imu_msg.angular_velocity, imu_msg.linear_acceleration])
    logging.info('Parsed the imu data!')

    return imu_data
