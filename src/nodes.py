#!/usr/bin/env python3

# Import configurations
from config import *

# Messages
from nav_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *

from tello_driver.msg import *
from h264_image_transport.msg import *

# Other imports
import os, bson, datetime
from tf.transformations import euler_from_quaternion

# ============== Callback function ============== #
# The function should always only have one variable 'data', where 'data' is the received message converted to a document

# Quaternion to euler callback
def debug(data) -> None:
    print(data)

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

NODES = [
    #############################################################
    # {
    #     'node'    : --The node name,
    #     'msg'     : --The type of message
    #     'sleep'   : --Sample time [optional, 1]
    #     'callback': --Callback function [optional, None]
    #     'dataPath': {
    #         'dataSource': --Name of data source in MongoDB [optional, config.DATASOURCE]
    #         'dataBase'  : --Name of data base in MongoDB [optional, config.DATALAKE]
    #         'collection': --Name of collection in MongoDB [optional, node name]
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
]