#!/usr/bin/env python3

# Import configurations
from config import *

# Import default callback funcitions
import defaultCallbacks as dc

# Messages
from nav_msgs.msg       import *
from sensor_msgs.msg    import *
from geometry_msgs.msg  import *
from ubiquity_motor.msg import *
from ros_monitoring.msg import *

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
        'topic'   : '/connectionStatus',
        'msg'     : SignalInformation,
    }, 
    # NodesStatus
    {
        'topic'   : '/nodesStatus',
        'msg'     : NodesInformation,
        'sleep'   : 5,
        'callback': dc.diffStore
    },
    # Odometry
    {
        'topic'   : '/odom',
        'msg'     : Odometry,
        'callback': dc.q2e,
    }, 
    # Battery
    {
        'topic'   : '/battery_state',
        'msg'     : BatteryState,
        'sleep'   : 5,
    }, 
    # LiDAR
    {
        'topic'   : '/scan',
        'msg'     : LaserScan,
        'sleep'   : 5,
    }, 
    # AMCL_pos
    {
        'topic'   : '/amcl_pose',
        'msg'     : PoseWithCovarianceStamped,
        'callback': dc.q2e,
    }, 
    # Motor state
    {
        'topic'   : '/motor_state',
        'msg'     : MotorState,
    },
    # Sonar
    {
        'topic'   : '/sonars',
        'msg'     : Range,
        'sleep'   : 5,
    }
]
