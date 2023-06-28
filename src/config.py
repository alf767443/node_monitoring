#!/usr/bin/env python3

import os, pymongo

#============ Storage ============#
PATH = os.path.expanduser('~')+'/tempData/'

#============ Mongo ============#
ROBOT_NAME = 'Tello_01'
DATALAKE   = "CeDRI_TelloEdu"
DATASOURCE = "CeDRI_robots"
CLIENT = pymongo.MongoClient('mongodb://192.168.217.183:27017/', 
                             connectTimeoutMS = 500, 
                             serverSelectionTimeoutMS = 1000, 
                             socketTimeoutMS = 500)
