#!/usr/bin/env python3

import os, pymongo

#============ Storage ============#
PATH = os.path.expanduser('~')+'/UGV_tempData/'

#============ Mongo ============#
DATALAKE = "New_Robot"
DATASOURCE = "CeDRI_robots"
CLIENT = pymongo.MongoClient('mongodb://192.168.217.183:27017/', 
                             connectTimeoutMS = 500, 
                             serverSelectionTimeoutMS = 1000, 
                             socketTimeoutMS = 500)
