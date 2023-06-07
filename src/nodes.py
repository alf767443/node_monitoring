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
def debug(data, node) -> None:
    print(data)

def q2e(data, node) -> None:
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
def diffStore(data, node) -> None:
    
    print(data)
    print(node)
    # Compare dictionaries
    def compare_dict(dict1, dict2):
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
    # Set file path and name, extension temporary JSON (.tjson)
    file = PATH + '/' + str(node['node'] + '.tjson')
    # Check if path exists
    if not os.path.exists(path=PATH):
        os.chmod
        os.makedirs(name=PATH)
    # Open file
    file = open(file=file, mode='bw+')
    _file = file.read()
    a = {'a':1}
    print(a.pop('a'))
    if _file == b'':
        print(None)
        # file.write(data)
    print(_file)
    # if not len(file):
    

    # _data = bson.BSON.decode(file.read())
    # try:
    #     _diag = bson.decode(file.read())
    #     _diag = _diag['status']
    # except:
    #     _diag = {}
    #     pass
    # diag = data['status']
    # _data = []
    # for diagnostics in diag:
    #     # Verifica se existe a key
    #     try:
    #         _diag[diagnostics['name']]
    #     except:
    #         _diag.update({diagnostics['name']: None})
    #     if (_diag[diagnostics['name']] != diagnostics['level']):
    #         _diag.update({diagnostics['name']: diagnostics['level']})
    #         diagnostics.update({'dateTime': datetime.datetime.now()})
    #         _data.append(diagnostics)
    # if len(_data) > 0:
    #     file.truncate(0)
    #     file.write(bson.encode(document={data}))
    # else:
    #     _data = None
    # file.close()
    # data = _data
    data = None
    file.close()

    
    
    return True

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
        'node'    : '/odom',
        'msg'     : Odometry,
        'sleep'   :  2,
        'callback': q2e,
    }, 
    # Battery
    {
        'node'    : '/battery_state',
        'msg'     : BatteryState,
        'sleep'   :  10,
    }, 
    # LiDAR
    {
        'node'    : '/scan',
        'msg'     : LaserScan,
        'sleep'   : 5,
    }, 
    # AMCL_pos
    {
        'node'    : '/amcl_pose',
        'msg'     : PoseWithCovarianceStamped,
        'sleep'   :  0.2,
        'callback': q2e,
    }, 
    # Motor state
    {
        'node'    : '/motor_state',
        'msg'     : MotorState,
        'sleep'   :  3,
    },
    # Sonar
    {
        'node'    : '/sonars',
        'msg'     : Range,
        'sleep'   : 5,
    }, 
    # ConnectionStatus
    {
        'node'    : '/connectionStatus',
        'msg'     : SignalInformation,
        'sleep'   : 5,
    }, 
    # NodesStatus
    {
        'node'    : '/nodesStatus',
        'msg'     : NodesInformation,
        'sleep'   : 5,
        'callback': diffStore
    },
]
