#!/usr/bin/env python3

# Import configurations
from config import *

# Messages
from nav_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from ros_monitoring.msg import *
from tello_driver.msg import *
from h264_image_transport.msg import *

# Other imports
import os, bson, datetime
from tf.transformations import euler_from_quaternion

# ============== Callback function ============== #
# The function should always only have one variable 'data', where 'data' is the received message converted to a document

# Debug topic callback
def debug(data, topic) -> None:
    print(data)
    print(topic)
    
# Store data just if is different
def diffStore(data, topic) -> None:
    # Create a local data
    _data = data.copy()
    _data.pop('dateTime')
    # Compare dictionaries
    def compare_dict(dict1, dict2)->bool:
        # Check size of the dicts
        if len(dict1) != len(dict2):
            return False
        # Check keys and values
        for key, value in dict1.items():
            if key not in dict2:
                return False
            # If is nested
            if isinstance(value, dict) and isinstance(dict2[key], dict):
                if not compare_dict(value, dict2[key]):
                    return False
            elif dict2[key] != value:
                return False
        return True
    # Set file path and name, extension temporary JSON (.tjson)
    file = PATH + str(topic['topic'].replace('/', '') + '.tjson')
    # Check if path exists
    if not os.path.exists(path=PATH):
        os.chmod
        os.makedirs(name=PATH)
    # Open file
    if not os.path.exists(path=file):
        file = open(file=file, mode='bw+')
    else:
        file = open(file=file, mode='br+')
    # Read the file
    _file = file.read()
    # Compare 'data' with the data in file
    if not _file == b'':
        # Decode the bson 
        try:
            _file = bson.BSON.decode(_file)
        except bson.errors.InvalidBSON:
            file.write(bson.encode(document=_data))
            file.close()
            return None            
        # Compare the dictionaries
        if compare_dict(_data, _file):
            data.clear()
        else:
            file.write(bson.encode(document=_data))
    # The file is void
    else:
        file.write(bson.encode(document=_data))
    # Close file
    file.close()

def q2e(data) -> None:
    # Get orientation
    orientation = data['pose']['pose']['orientation']
    # Convert
    (raw, pitch, yaw) = euler_from_quaternion([orientation['x'], orientation['y'], orientation['z'], orientation['w']])
    # Add in a dictionary
    orientation = {
        'raw'     :  raw,
        'pitch'   : pitch,
        'yaw'     : yaw,
    }
    # Update the data to storage
    data.update({'pose': {'pose': {'position': data['pose']['pose']['position'], 'orientation': orientation}}})


# ============== Nodes ============== #

TOPICS = [
    #############################################################
    # {
    #     'topic'    : --The topic name,
    #     'msg'     : --The type of message
    #     'sleep'   : --Sample time [optional, 1]
    #     'callback': --Callback function [optional, None]
    #     'dataPath': {
    #         'dataSource': --Name of data source in MongoDB [optional, config.DATASOURCE]
    #         'dataBase'  : --Name of data base in MongoDB [optional, config.DATALAKE]
    #         'collection': --Name of collection in MongoDB [optional, topic name]
    #     }
    # }
    #############################################################
   
    # Odometry
    {
        'node'    : '/tello/odom',
        'msg'     : Odometry,
        'callback': q2e,
    },
    # Statues
    {
        'node'    : '/tello/status',
        'msg'     : TelloStatus,
    },
    # Inertial Sensor Modules
    {
        'node'    : '/tello/imu',
        'msg'     : Imu,
    },
    # Camera info
    {
        'node'    : '/tello/image_raw/camera_info',
        'msg'     : CameraInfo,
    },
    # Image RAW
    {
        'node'    : '/tello/image_raw/h264',
        'msg'     : H264Packet,
    },
    # ConnectionStatus
    {
        'topic'    : '/connectionStatus',
        'msg'     : SignalInformation,
        'sleep'   : 5,
    }, 
    # NodesStatus
    {
        'topic'    : '/nodesStatus',
        'msg'     : NodesInformation,
        'sleep'   : 5,
        'callback': diffStore
    }
]
