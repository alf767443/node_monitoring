#!/usr/bin/env python3

# Import configurations
from config import *

# Import default callback funcitions
import defaultCallbacks as dc

# Messages
from nav_msgs.msg             import *
from sensor_msgs.msg          import *
from geometry_msgs.msg        import *
from ros_monitoring.msg       import *
from ros_monitoring.msg       import *
from tello_driver.msg         import *
from h264_image_transport.msg import *

# Other imports
import os, bson, datetime

# ============== Callback function ============== #
# The function should always only have one variable 'data', where 'data' is the received message converted to a document
 
 #############################################################
 # The callback function must contain the 'data' variable that 
 # will receive the message, plus have a topic variable 
 # which must not be manipulated
 
 # def debug(data, topic) -> None:
 #     print(data)
 #############################################################

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
        'callback': dc.diffStore
    }
    # Odometry
    {
        'node'    : '/tello/odom',
        'msg'     : Odometry,
        'callback': dc.q2e,
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
    }
]
