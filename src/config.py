#!/usr/bin/env python3

import os, pymongo

#============ Storage ============#
PATH = os.path.expanduser('~')+'/UGV_tempData/'

#============ Mongo ============#
DATALAKE = "datalake_UGV_Magni_debug"
DATASOURCE = "CeDRI_robots"
CLIENT = pymongo.MongoClient('mongodb://192.168.217.183:27017/', 
                             connectTimeoutMS = 100, 
                             serverSelectionTimeoutMS = 100, 
                             socketTimeoutMS = 100)
