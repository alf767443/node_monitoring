#!/usr/bin/env python3

# Import configurations
from config import *

# Import default callback funcitions
import defaultCallbacks as dc

# Messages
from nav_msgs.msg               import *
from sensor_msgs.msg            import *
from geometry_msgs.msg          import *
from dynamic_reconfigure.msg    import *
from theora_image_transport.msg import *
from rosgraph_msgs.msg          import *
from diagnostic_msgs.msg        import *
from tf2_msgs.msg               import *
from gazebo_msgs.msg            import *
from map_msgs.msg               import *
from actionlib_msgs.msg         import *
from move_base_msgs.msg         import *
from visualization_msgs.msg     import *
from std_msgs.msg               import *

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
    # /amcl_pose
    {
        'topic'   : '/amcl_pose',
        'msg'     : PoseWithCovarianceStamped,
        'sleep'   : 0.2
    },
    # /camera/camera_info
    {
        'topic'   : '/camera/camera_info',
        'msg'     : CameraInfo,
        'sleep'   : 0.2
    },
    ## Listening to the image compressed
    # /camera/image_raw
    # {
    #     'topic'   : '/camera/image_raw',
    #     'msg'     : Image,
    #     'sleep'   : 0.2
    # },
    # /camera/image_raw/compressed
    {
        'topic'   : '/camera/image_raw/compressed',
        'msg'     : CompressedImage,
        'sleep'   : 0.2
    },
    # /camera/image_raw/compressed/parameter_descriptions
    {
        'topic'   : '/camera/image_raw/compressed/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /camera/image_raw/compressed/parameter_updates
    {
        'topic'   : '/camera/image_raw/compressed/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /camera/image_raw/compressedDepth
    {
        'topic'   : '/camera/image_raw/compressedDepth',
        'msg'     : CompressedImage,
        'sleep'   : 0.2
    },
    # /camera/image_raw/compressedDepth/parameter_descriptions
    {
        'topic'   : '/camera/image_raw/compressedDepth/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /camera/image_raw/compressedDepth/parameter_updates
    {
        'topic'   : '/camera/image_raw/compressedDepth/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /camera/image_raw/theora
    {
        'topic'   : '/camera/image_raw/theora',
        'msg'     : Packet,
        'sleep'   : 0.2
    },
    # /camera/image_raw/theora/parameter_descriptions
    {
        'topic'   : '/camera/image_raw/theora/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /camera/image_raw/theora/parameter_updates
    {
        'topic'   : '/camera/image_raw/theora/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /camera/parameter_descriptions
    {
        'topic'   : '/camera/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /camera/parameter_updates
    {
        'topic'   : '/camera/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /camera/realsense/color/camera_info
    {
        'topic'   : '/camera/realsense/color/camera_info',
        'msg'     : CameraInfo,
        'sleep'   : 0.2
    },
    # /camera/realsense/color/image_raw
    {
        'topic'   : '/camera/realsense/color/image_raw',
        'msg'     : Image,
        'sleep'   : 0.2
    },
    # /camera/realsense/color/image_raw/compressed
    {
        'topic'   : '/camera/realsense/color/image_raw/compressed',
        'msg'     : CompressedImage,
        'sleep'   : 0.2
    },
    # /camera/realsense/color/image_raw/compressed/parameter_descriptions
    {
        'topic'   : '/camera/realsense/color/image_raw/compressed/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /camera/realsense/color/image_raw/compressed/parameter_updates
    {
        'topic'   : '/camera/realsense/color/image_raw/compressed/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /camera/realsense/color/image_raw/compressedDepth
    {
        'topic'   : '/camera/realsense/color/image_raw/compressedDepth',
        'msg'     : CompressedImage,
        'sleep'   : 0.2
    },
    # /camera/realsense/color/image_raw/compressedDepth/parameter_descriptions
    {
        'topic'   : '/camera/realsense/color/image_raw/compressedDepth/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /camera/realsense/color/image_raw/compressedDepth/parameter_updates
    {
        'topic'   : '/camera/realsense/color/image_raw/compressedDepth/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /camera/realsense/color/image_raw/theora
    {
        'topic'   : '/camera/realsense/color/image_raw/theora',
        'msg'     : Packet,
        'sleep'   : 0.2
    },
    # /camera/realsense/color/image_raw/theora/parameter_descriptions
    {
        'topic'   : '/camera/realsense/color/image_raw/theora/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /camera/realsense/color/image_raw/theora/parameter_updates
    {
        'topic'   : '/camera/realsense/color/image_raw/theora/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /camera/realsense/depth/camera_info
    {
        'topic'   : '/camera/realsense/depth/camera_info',
        'msg'     : CameraInfo,
        'sleep'   : 0.2
    },
    # /camera/realsense/depth/color/points
    {
        'topic'   : '/camera/realsense/depth/color/points',
        'msg'     : PointCloud2,
        'sleep'   : 0.2
    },
    # /camera/realsense/depth/image_rect_raw
    {
        'topic'   : '/camera/realsense/depth/image_rect_raw',
        'msg'     : Image,
        'sleep'   : 0.2
    },
    # /camera/realsense/parameter_descriptions
    {
        'topic'   : '/camera/realsense/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /camera/realsense/parameter_updates
    {
        'topic'   : '/camera/realsense/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /clicked_point
    {
        'topic'   : '/clicked_point',
        'msg'     : PointStamped,
        'sleep'   : 0.2
    },
    # /clock
    {
        'topic'   : '/clock',
        'msg'     : Clock,
        'sleep'   : 0.2
    },
    # /cmd_vel
    {
        'topic'   : '/cmd_vel',
        'msg'     : Twist,
        'sleep'   : 0.2
    },
    # /diagnostics
    {
        'topic'   : '/diagnostics',
        'msg'     : DiagnosticArray,
        'sleep'   : 0.2
    },
    # /e_stop
    {
        'topic'   : '/e_stop',
        'msg'     : Bool,
        'sleep'   : 0.2
    },
    # /gazebo/link_states
    {
        'topic'   : '/gazebo/link_states',
        'msg'     : LinkStates,
        'sleep'   : 0.2
    },
    # /gazebo/model_states
    {
        'topic'   : '/gazebo/model_states',
        'msg'     : ModelStates,
        'sleep'   : 0.2
    },
    # /gazebo/parameter_descriptions
    {
        'topic'   : '/gazebo/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /gazebo/parameter_updates
    {
        'topic'   : '/gazebo/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /gazebo/performance_metrics
    {
        'topic'   : '/gazebo/performance_metrics',
        'msg'     : PerformanceMetrics,
        'sleep'   : 0.2
    },
    # /gazebo/set_link_state
    {
        'topic'   : '/gazebo/set_link_state',
        'msg'     : LinkState,
        'sleep'   : 0.2
    },
    # /gazebo/set_model_state
    {
        'topic'   : '/gazebo/set_model_state',
        'msg'     : ModelState,
        'sleep'   : 0.2
    },
    # /husky_velocity_controller/cmd_vel
    {
        'topic'   : '/husky_velocity_controller/cmd_vel',
        'msg'     : Twist,
        'sleep'   : 0.2
    },
    # /husky_velocity_controller/odom
    {
        'topic'   : '/husky_velocity_controller/odom',
        'msg'     : Odometry,
        'sleep'   : 0.2
    },
    # /husky_velocity_controller/parameter_descriptions
    {
        'topic'   : '/husky_velocity_controller/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /husky_velocity_controller/parameter_updates
    {
        'topic'   : '/husky_velocity_controller/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /imu/data
    {
        'topic'   : '/imu/data',
        'msg'     : Imu,
        'sleep'   : 0.2
    },
    # /imu/data/accel/parameter_descriptions
    {
        'topic'   : '/imu/data/accel/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /imu/data/accel/parameter_updates
    {
        'topic'   : '/imu/data/accel/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /imu/data/bias
    {
        'topic'   : '/imu/data/bias',
        'msg'     : Imu,
        'sleep'   : 0.2
    },
    # /imu/data/rate/parameter_descriptions
    {
        'topic'   : '/imu/data/rate/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /imu/data/rate/parameter_updates
    {
        'topic'   : '/imu/data/rate/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /imu/data/yaw/parameter_descriptions
    {
        'topic'   : '/imu/data/yaw/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /imu/data/yaw/parameter_updates
    {
        'topic'   : '/imu/data/yaw/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /initialpose
    {
        'topic'   : '/initialpose',
        'msg'     : PoseWithCovarianceStamped,
        'sleep'   : 0.2
    },
    # /joint_states
    {
        'topic'   : '/joint_states',
        'msg'     : JointState,
        'sleep'   : 0.2
    },
    # /joy_teleop/cmd_vel
    {
        'topic'   : '/joy_teleop/cmd_vel',
        'msg'     : Twist,
        'sleep'   : 0.2
    },
    # /joy_teleop/joy
    {
        'topic'   : '/joy_teleop/joy',
        'msg'     : Joy,
        'sleep'   : 0.2
    },
    # /joy_teleop/joy/set_feedback
    {
        'topic'   : '/joy_teleop/joy/set_feedback',
        'msg'     : JoyFeedbackArray,
        'sleep'   : 0.2
    },
    # /lidar/particle_cloud
    {
        'topic'   : '/lidar/particle_cloud',
        'msg'     : PointCloud2,
        'sleep'   : 0.2
    },
    ## File oversize
    # /map
    # {
    #     'topic'   : '/map',
    #     'msg'     : OccupancyGrid,
    #     'sleep'   : 0.2
    # },
    # /map_metadata
    {
        'topic'   : '/map_metadata',
        'msg'     : MapMetaData,
        'sleep'   : 0.2
    },
    # /map_updates
    {
        'topic'   : '/map_updates',
        'msg'     : OccupancyGridUpdate,
        'sleep'   : 0.2
    },
    # /move_base/DWAPlannerROS/cost_cloud
    {
        'topic'   : '/move_base/DWAPlannerROS/cost_cloud',
        'msg'     : PointCloud2,
        'sleep'   : 0.2
    },
    # /move_base/DWAPlannerROS/global_plan
    {
        'topic'   : '/move_base/DWAPlannerROS/global_plan',
        'msg'     : Path,
        'sleep'   : 0.2
    },
    # /move_base/DWAPlannerROS/local_plan
    {
        'topic'   : '/move_base/DWAPlannerROS/local_plan',
        'msg'     : Path,
        'sleep'   : 0.2
    },
    # /move_base/DWAPlannerROS/parameter_descriptions
    {
        'topic'   : '/move_base/DWAPlannerROS/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /move_base/DWAPlannerROS/parameter_updates
    {
        'topic'   : '/move_base/DWAPlannerROS/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /move_base/DWAPlannerROS/trajectory_cloud
    {
        'topic'   : '/move_base/DWAPlannerROS/trajectory_cloud',
        'msg'     : PointCloud2,
        'sleep'   : 0.2
    },
    # /move_base/GraphPlanner/expand
    {
        'topic'   : '/move_base/GraphPlanner/expand',
        'msg'     : OccupancyGrid,
        'sleep'   : 0.2
    },
    # /move_base/GraphPlanner/plan
    {
        'topic'   : '/move_base/GraphPlanner/plan',
        'msg'     : Path,
        'sleep'   : 0.2
    },
    # /move_base/cancel
    {
        'topic'   : '/move_base/cancel',
        'msg'     : GoalID,
        'sleep'   : 0.2
    },
    # /move_base/current_goal
    {
        'topic'   : '/move_base/current_goal',
        'msg'     : PoseStamped,
        'sleep'   : 0.2
    },
    # /move_base/feedback
    {
        'topic'   : '/move_base/feedback',
        'msg'     : MoveBaseActionFeedback,
        'sleep'   : 0.2
    },
    ## File oversize
    # /move_base/global_costmap/costmap
    # {
    #     'topic'   : '/move_base/global_costmap/costmap',
    #     'msg'     : OccupancyGrid,
    #     'sleep'   : 0.2
    # },
    # /move_base/global_costmap/costmap_updates
    {
        'topic'   : '/move_base/global_costmap/costmap_updates',
        'msg'     : OccupancyGridUpdate,
        'sleep'   : 0.2
    },
    # /move_base/global_costmap/footprint
    {
        'topic'   : '/move_base/global_costmap/footprint',
        'msg'     : PolygonStamped,
        'sleep'   : 0.2
    },
    # /move_base/global_costmap/inflation/parameter_descriptions
    {
        'topic'   : '/move_base/global_costmap/inflation/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /move_base/global_costmap/inflation/parameter_updates
    {
        'topic'   : '/move_base/global_costmap/inflation/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /move_base/global_costmap/parameter_descriptions
    {
        'topic'   : '/move_base/global_costmap/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /move_base/global_costmap/parameter_updates
    {
        'topic'   : '/move_base/global_costmap/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /move_base/global_costmap/static/parameter_descriptions
    {
        'topic'   : '/move_base/global_costmap/static/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /move_base/global_costmap/static/parameter_updates
    {
        'topic'   : '/move_base/global_costmap/static/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /move_base/goal
    {
        'topic'   : '/move_base/goal',
        'msg'     : MoveBaseActionGoal,
        'sleep'   : 0.2
    },
    # /move_base/local_costmap/costmap
    {
        'topic'   : '/move_base/local_costmap/costmap',
        'msg'     : OccupancyGrid,
        'sleep'   : 0.2
    },
    # /move_base/local_costmap/costmap_updates
    {
        'topic'   : '/move_base/local_costmap/costmap_updates',
        'msg'     : OccupancyGridUpdate,
        'sleep'   : 0.2
    },
    # /move_base/local_costmap/footprint
    {
        'topic'   : '/move_base/local_costmap/footprint',
        'msg'     : PolygonStamped,
        'sleep'   : 0.2
    },
    # /move_base/local_costmap/inflation/parameter_descriptions
    {
        'topic'   : '/move_base/local_costmap/inflation/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /move_base/local_costmap/inflation/parameter_updates
    {
        'topic'   : '/move_base/local_costmap/inflation/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /move_base/local_costmap/obstacles_laser/parameter_descriptions
    {
        'topic'   : '/move_base/local_costmap/obstacles_laser/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /move_base/local_costmap/obstacles_laser/parameter_updates
    {
        'topic'   : '/move_base/local_costmap/obstacles_laser/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /move_base/local_costmap/parameter_descriptions
    {
        'topic'   : '/move_base/local_costmap/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /move_base/local_costmap/parameter_updates
    {
        'topic'   : '/move_base/local_costmap/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /move_base/parameter_descriptions
    {
        'topic'   : '/move_base/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /move_base/parameter_updates
    {
        'topic'   : '/move_base/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /move_base/recovery_status
    {
        'topic'   : '/move_base/recovery_status',
        'msg'     : RecoveryStatus,
        'sleep'   : 0.2
    },
    # /move_base/result
    {
        'topic'   : '/move_base/result',
        'msg'     : MoveBaseActionResult,
        'sleep'   : 0.2
    },
    # /move_base/status
    {
        'topic'   : '/move_base/status',
        'msg'     : GoalStatusArray,
        'sleep'   : 0.2
    },
    # /move_base_simple/goal
    {
        'topic'   : '/move_base_simple/goal',
        'msg'     : PoseStamped,
        'sleep'   : 0.2
    },
    # /navsat/fix
    {
        'topic'   : '/navsat/fix',
        'msg'     : NavSatFix,
        'sleep'   : 0.2
    },
    # /navsat/fix/position/parameter_descriptions
    {
        'topic'   : '/navsat/fix/position/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /navsat/fix/position/parameter_updates
    {
        'topic'   : '/navsat/fix/position/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /navsat/fix/status/parameter_descriptions
    {
        'topic'   : '/navsat/fix/status/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /navsat/fix/status/parameter_updates
    {
        'topic'   : '/navsat/fix/status/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /navsat/fix/velocity/parameter_descriptions
    {
        'topic'   : '/navsat/fix/velocity/parameter_descriptions',
        'msg'     : ConfigDescription,
        'sleep'   : 0.2
    },
    # /navsat/fix/velocity/parameter_updates
    {
        'topic'   : '/navsat/fix/velocity/parameter_updates',
        'msg'     : Config,
        'sleep'   : 0.2
    },
    # /navsat/vel
    {
        'topic'   : '/navsat/vel',
        'msg'     : Vector3Stamped,
        'sleep'   : 0.2
    },
    # /odom
    {
        'topic'   : '/odom',
        'msg'     : Odometry,
        'sleep'   : 0.2
    },
    # /odometry/filtered
    {
        'topic'   : '/odometry/filtered',
        'msg'     : Odometry,
        'sleep'   : 0.2
    },
    # /particlecloud
    {
        'topic'   : '/particlecloud',
        'msg'     : PoseArray,
        'sleep'   : 0.2
    },
    # /rosout
    {
        'topic'   : '/rosout',
        'msg'     : Log,
        'sleep'   : 0.2
    },
    # /rosout_agg
    {
        'topic'   : '/rosout_agg',
        'msg'     : Log,
        'sleep'   : 0.2
    },
    # /scan
    {
        'topic'   : '/scan',
        'msg'     : LaserScan,
        'sleep'   : 0.2
    },
    # /set_pose
    {
        'topic'   : '/set_pose',
        'msg'     : PoseWithCovarianceStamped,
        'sleep'   : 0.2
    },
    # /slam_gmapping/entropy
    {
        'topic'   : '/slam_gmapping/entropy',
        'msg'     : Float64,
        'sleep'   : 0.2
    },
    # /tf
    {
        'topic'   : '/tf',
        'msg'     : TFMessage,
        'sleep'   : 0.2
    },
    # /tf_static
    {
        'topic'   : '/tf_static',
        'msg'     : TFMessage,
        'sleep'   : 0.2
    },
    # /twist_marker_server/cmd_vel
    {
        'topic'   : '/twist_marker_server/cmd_vel',
        'msg'     : Twist,
        'sleep'   : 0.2
    },
    # /twist_marker_server/feedback
    {
        'topic'   : '/twist_marker_server/feedback',
        'msg'     : InteractiveMarkerFeedback,
        'sleep'   : 0.2
    },
    # /twist_marker_server/update
    {
        'topic'   : '/twist_marker_server/update',
        'msg'     : InteractiveMarkerUpdate,
        'sleep'   : 0.2
    },
    # /twist_marker_server/update_full
    {
        'topic'   : '/twist_marker_server/update_full',
        'msg'     : InteractiveMarkerInit,
        'sleep'   : 0.2
    },
]