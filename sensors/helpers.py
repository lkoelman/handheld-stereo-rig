from os import makedirs
from os.path import join

from datetime import datetime

# File recording place
def set_sensor_recording_folder(sensor_name = "sensor"):
    # Initialise the directory name 
    date = datetime.now().strftime("%Y_%m_%d-%I:%M:%S")
    dir_name = "/tmp" + "/vn-100" + "/" + date 
    # Create the directory
    makedirs(dir_name, exist_ok=True)
    # Set the filename
    filepath = join(dir_name, f'{sensor_name}-{date}.pkl')
    return filepath 
        