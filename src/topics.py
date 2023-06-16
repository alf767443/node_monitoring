#!/usr/bin/env python3

# Import configurations
from config import *

# Messages
from nav_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from ubiquity_motor.msg import *
from ros_monitoring.msg import *

# Other imports
import os, bson, datetime
from tf.transformations import euler_from_quaternion

# ============== Callback function ============== #
# The function should always only have one variable 'data', where 'data' is the received message converted to a document

# Quaternion to euler callback
def debug(data, topic) -> None:
    print(data)

def q2e(data, topic) -> None:
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

# Store data just if is 
def diffStore(data, topic) -> None:
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
    # Write a BSON in a binary file
    def writeBfile(data, path):
        file = open(file=path, mode='bw+')
        file.write(bson.encode(document=data))
        file.close()

    # Create a local data
    _data = data.copy()
    _data.pop('dateTime')
    # Set file path and name, extension temporary JSON (.tjson)
    filePath = PATH + str(topic['topic'].replace('/', '') + '.tjson')
    # Check if path exists
    if not os.path.exists(path=PATH):
        os.chmod
        os.makedirs(name=PATH)
    # Open file
    if not os.path.exists(path=filePath):
        file = open(file=filePath, mode='bw+')
    else:
        file = open(file=filePath, mode='br+')
    # Read the file
    _file = file.read()
    file.close()
    # Compare 'data' with the data in file
    if not _file == b'':
        # Decode the bson 
        try:
            _file = bson.BSON.decode(_file)
        except bson.errors.InvalidBSON:
            writeBfile(data=_data, path=filePath)
            return None            
        # Compare the dictionaries
        if compare_dict(_data, _file):
            data.clear()
        else:
            writeBfile(data=_data, path=filePath)
    # The file is void
    else:
        writeBfile(data=_data, path=filePath)
    # Close file
    file.close()

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
        'callback': diffStore
    },
    # Odometry
    {
        'topic'    : '/odom',
        'msg'     : Odometry,
        'sleep'   :  2,
        'callback': q2e,
    }, 
    # Battery
    {
        'topic'    : '/battery_state',
        'msg'     : BatteryState,
        'sleep'   :  10,
    }, 
    # LiDAR
    {
        'topic'    : '/scan',
        'msg'     : LaserScan,
        'sleep'   : 5,
    }, 
    # AMCL_pos
    {
        'topic'    : '/amcl_pose',
        'msg'     : PoseWithCovarianceStamped,
        'sleep'   :  0.2,
        'callback': q2e,
    }, 
    # Motor state
    {
        'topic'    : '/motor_state',
        'msg'     : MotorState,
        'sleep'   :  3,
    },
    # Sonar
    {
        'topic'    : '/sonars',
        'msg'     : Range,
        'sleep'   : 5,
    }
]