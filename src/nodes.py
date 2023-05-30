#!/usr/bin/env python3

# Import configurations
from config import DATALAKE, DATASOURCE, PATH

# Messages
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseWithCovarianceStamped
from ubiquity_motor.msg import MotorState
from ros_monitoring.msg import SignalInformation

# Other imports
import os, bson, datetime
from tf.transformations import euler_from_quaternion

# ============== Callback function ============== #
# The function should always only have one variable 'data', where 'data' is the received message converted to a document

# Quaternion to euler callback
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

# To test
def diag(data) -> None:
    path =  PATH + "temp"
    file = "/diag.bjson"
    if not os.path.exists(path=path):
        os.chmod
        os.makedirs(name=path)
    file = open(file=path + file, mode='bw+')
    # Create directory if it don't exist
    try:
        _diag = bson.decode(file.read())
        _diag = _diag['status']
    except:
        _diag = {}
        pass
    diag = data['status']
    _data = []
    for diagnostics in diag:
        # Verifica se existe a chave
        try:
            _diag[diagnostics['name']]
        except:
            _diag.update({diagnostics['name']: None})
        if (_diag[diagnostics['name']] != diagnostics['level']):
            _diag.update({diagnostics['name']: diagnostics['level']})
            diagnostics.update({'dateTime': datetime.datetime.now()})
            _data.append(diagnostics)
    if len(_data) > 0:
        file.truncate(0)
        file.write(bson.encode(document={data}))
    else:
        _data = None
    file.close()
    data = _data

# ============== Nodes ============== #

NODES = [
    #############################################################
    # {
    #     'node'    : --The node name (odom),
    #     'msg'     : --The type of message
    #     'sleep'   : --Sample time
    #     'callback': --Callback function 
    #     'dataPath': {
    #         'dataSource': --Name of data source in MongoDB
    #         'dataBase'  : --Name of data base in MongoDB
    #         'collection': --Name of collection in MongoDB
    #     }
    # }
    #############################################################

    # Odometry
    {
        'node'    : 'odom',
        'msg'     : Odometry,
        'sleep'    : 2,
        'callback': q2e,
        'dataPath': {
            'dataSource': DATASOURCE, 
            'dataBase'  : DATALAKE,
            'collection': 'odom'
        }
    }, 
    # Battery
    {
        'node'    : 'battery_state',
        'msg'     : BatteryState,
        'sleep'    : 10,
        'callback': None,
        'dataPath': {
            'dataSource': DATASOURCE, 
            'dataBase'  : DATALAKE,
            'collection': 'battery_state'
        }
    }, 
    # LiDAR
    {
        'node'    : 'scan',
        'msg'     : LaserScan,
        'sleep'    : 5,
        'callback': None,
        'dataPath': {
            'dataSource': DATASOURCE, 
            'dataBase'  : DATALAKE,
            'collection': 'scan'
        }
    }, 
    # AMCL_pos
    {
        'node'    : 'amcl_pose',
        'msg'     : PoseWithCovarianceStamped,
        'sleep'    : 0.2,
        'callback': q2e,
        'dataPath': {
            'dataSource': DATASOURCE, 
            'dataBase'  : DATALAKE,
            'collection': 'amcl_pose'
        }
    }, 
    # Motor state
    {
        'node'    : 'motor_state',
        'msg'     : MotorState,
        'sleep'    : 3,
        'callback': None,
        'dataPath': {
            'dataSource': DATASOURCE, 
            'dataBase'  : DATALAKE,
            'collection': 'motor_state'
        }
    }, 
    # Occupancy map
    {
        'node'    : 'map',
        'msg'     : OccupancyGrid,
        'sleep'    : 20,
        'callback': None,
        'dataPath': {
            'dataSource': DATASOURCE, 
            'dataBase'  : DATALAKE,
            'collection': 'map'
        }
    }, 
    # Sonar
    {
        'node'    : 'sonars',
        'msg'     : Range,
        'sleep'    : 5,
        'callback': None,
        'dataPath': {
            'dataSource': DATASOURCE, 
            'dataBase'  : DATALAKE,
            'collection': 'sonars'
        }
    }, 
    # ConnectionStatus
    {
        'node'    : 'connectionStatus',
        'msg'     : SignalInformation,
        'sleep'    : 5,
        'callback': None,
        'dataPath': {
            'dataSource': DATASOURCE, 
            'dataBase'  : DATALAKE,
            'collection': 'connectionStatus'
        }
    }, 
]
