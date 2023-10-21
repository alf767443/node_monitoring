#!/usr/bin/env python3

import os, pymongo

#============ Storage ============#
PATH = os.path.expanduser('~')+'/tempData/'

#============ Mongo ============#
ROBOT_NAME = 'Husky_Robot_01'
DATALAKE = "CeDRI_Husky_Sim"
DATASOURCE = "CeDRI_robots"
CLIENT = pymongo.MongoClient('mongodb://127.0.0.1:27017/', 
                             connectTimeoutMS = 500, 
                             serverSelectionTimeoutMS = 1000, 
                             socketTimeoutMS = 500)
