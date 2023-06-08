# node_monitoring

This ROS package is intended to collect data from topics, transform, store, manage and send them to a MongoDB server. Being designed to maintain the collection even in connection drops with the server.



# Install

To install this ROS package:

	cd ~/catkin_ws/src
	git clone https://github.com/alf767443/node_monitoring.git
	sudo chmod +x src/bufferManager.py src/listenNode.py
	cd ..
	catkin_make

Installation of **ros_monitoring** is recommended. Check the link:
>https://github.com/alf767443/ros_monitoring.git

## Mapped robots

There are some robots with some of their nodes already mapped. This way you can substitute the line

>~/catkin_ws/src$ git clone https://github.com/alf767443/node_monitoring.git

as described in the subtopics below:

## Magni Silver robotic platform of CeDRI

It is a brach created for the AMR developed by the Research Centre in Digitalization and Intelligent Robotics (_CeDRI_), based on the Magni Silver robotics platform from Ubiquity Robotics.
	
	git clone -b MagniSilver_CeDRI  https://github.com/alf767443/node_monitoring.git
### Configuration
>ROBOT_NAME  =  'bigCedDRI'
>DATALAKE   =  'CeDRI_Magni'
>
### Topics
 - /amcl_pose
 - /battery_state
 - /connectionStatus
 - /motor_state
 - /nodesStatus
 - /odom
 - /scan
 - /sonars


## TelloEdu of CeDRI

This branch was created for the DJI TelloEdu drone, which uses *tello_drive*.
	
	git clone -b TelloEdu  https://github.com/alf767443/node_monitoring.git
	
### Configuration
>ROBOT_NAME  =  'Tello_01'
>DATALAKE   =  'CeDRI_TelloEdu'
>
### Topics
 - /connectionStatus
 - /nodesStatus
 - /tello/imu
 - /tello/image_raw/camera_info
 - /tello/image_raw/h264
 - /tello/odom
 - /tello/status




# Topics (*topics.py*)

This file is a configuration file in which are described the topics that will be subscribed (**topic**), the message type (**msg**), the waiting time between captures (**sleep**), data manipulation function (**callback**), storage information (**dataSource, dataBase, collection**).
## Structure

	{
	'topic'   : --The topic name,
	'msg'     : --The type of message
	'sleep'   : --Sample time [optional, 1]
	'callback': --Callback function [optional, None]
	'dataPath': {
		'dataSource': --Name of data source in MongoDB [optional, config.DATASOURCE]
		'dataBase'  : --Name of data base in MongoDB [optional, config.DATALAKE]
		'collection': --Name of collection in MongoDB [optional, topic name]
		}
	}

# Configurations (config.py)

This file contains the settings of values robot name (**ROBOT_NAME**), database (**DATALAKE**), MongoDB server address (**CLIENT**) and buffer path (**PATH**).
## Structure
	PATH        = os.path.expanduser('~')+'/tempData/'
	ROBOT_NAME  = 'New Robot'
	DATALAKE    = "CeDRI_New_Robot"
	CLIENT      = pymongo.MongoClient('mongodb://192.168.217.183:27017/')

# More information

This package was developed as part of the thesis for Master in Industrial Engineering with specialization in Electrical Engineering at Polytechnic Institute of Bragança (_IPB_), the work was developed at the Research Centre in Digitalization and Intelligent Robotics (_CeDRI_).
The project consists of three repositories with the links below:
	
>https://github.com/alf767443/node_monitoring

>https://github.com/alf767443/ros_monitoring

>https://github.com/alf767443/UGV-dashboard
