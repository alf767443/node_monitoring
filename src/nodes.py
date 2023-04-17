# Global imports
from GlobalSets.Mongo import DataSource as Source, DataBases as db, Collections as col

from config import DATALAKE, DATASOURCE

# Messages
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseWithCovarianceStamped
from ubiquity_motor.msg import MotorState

from diagnostic_msgs.msg import DiagnosticArray
from dynamic_reconfigure.msg import ConfigDescription

# Modifications
from tf.transformations import euler_from_quaternion


def q2e(data) -> None:
    orientation = data['pose']['pose']['orientation']
    (raw, pitch, yaw) = euler_from_quaternion([orientation['x'], orientation['y'], orientation['z'], orientation['w']])
    orientation = {
        'raw'     :  raw,
        'pitch'   : pitch,
        'yaw'     : yaw,
    }
    data.update({'pose': {'pose': {'position': data['pose']['pose']['position'], 'orientation': orientation}}})


NODES = [
    #############################################################
    # {
    #     'node'    : --The node name (odom),
    #     'msg'     : --The type of message
    #     'rate'    : --Sample rate
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
        'rate'    : 0.2,
        'callback': q2e,
        'dataPath': {
            'dataSource': DATASOURCE, 
            'dataBase'  : DATALAKE,
            'collection': 'Position_Odometry'
        }
    }, 
    # Battery
    {
        'node'    : 'battery_state',
        'msg'     : BatteryState,
        'rate'    : 0.2,
        'callback': None,
        'dataPath': {
            'dataSource': DATASOURCE, 
            'dataBase'  : DATALAKE,
            'collection': 'Battery'
        }
    }, 
    # LiDAR
    {
        'node'    : 'scan',
        'msg'     : LaserScan,
        'rate'    : 0.2,
        'callback': None,
        'dataPath': {
            'dataSource': DATASOURCE, 
            'dataBase'  : DATALAKE,
            'collection': 'LiDAR'
        }
    }, 
    # AMCL_pos
    {
        'node'    : 'amcl_pose',
        'msg'     : PoseWithCovarianceStamped,
        'rate'    : 0.2,
        'callback': q2e,
        'dataPath': {
            'dataSource': DATASOURCE, 
            'dataBase'  : DATALAKE,
            'collection': 'Position_AMCL'
        }
    }, 
    # Motor state
    {
        'node'    : 'motor_state',
        'msg'     : MotorState,
        'rate'    : 0.2,
        'callback': None,
        'dataPath': {
            'dataSource': DATASOURCE, 
            'dataBase'  : DATALAKE,
            'collection': 'Motor'
        }
    }, 
    # Occupancy map
    {
        'node'    : 'map',
        'msg'     : OccupancyGrid,
        'rate'    : 0.01,
        'callback': None,
        'dataPath': {
            'dataSource': DATASOURCE, 
            'dataBase'  : DATALAKE,
            'collection': 'Occupancy'
        }
    },    
]
