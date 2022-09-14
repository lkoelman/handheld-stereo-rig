import cv2
import numpy as np
import os
from tqdm import tqdm

image_save_dir = '/home/jpdemir/c-hopper/data/edr_demo/image_data'

    # Skip the first few frames until camera settles up
kNumberOfFramesToSkip = 7

'''Saves the Color Image data as from Ros bag png'''
def SaveColorImageAsPng(image_msg_generator, number_of_messages, downsample_rate):
    # Change the current directory 
    # to specified directory 
    os.chdir(image_save_dir)
    cntr = 0
    for i in tqdm(range(number_of_messages)):
        _, image_msg, image_t = next(image_msg_generator)
        
        if(i < kNumberOfFramesToSkip):
            continue
            
        if((i % downsample_rate) != 0):
            continue
        
        filename = 'rgb_' + str(cntr)+'.png'    
        cntr+=1
        
        image_data = np.frombuffer(image_msg.data, np.uint8)
        image_data_reshaped = np.reshape(image_data, (image_msg.height, image_msg.width, 3))
        # png likes bgr so we will leave it as it is
        cv2.imwrite(filename, image_data_reshaped)
        
    logging.info('Converted the camera data!')
    return color_image_data