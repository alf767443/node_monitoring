#!/usr/bin/env python3

# Import config
from config import PATH, CLIENT

import rospy, os, bson
from pymongo import errors as pymongo_erros
from bson import errors as bson_errors


class bufferManager():
    def __init__(self) -> None:
        # Starts unique node in the ROS core with the name bufferManager
        rospy.init_node('bufferManager', anonymous=False)
        # Node rate
        rate = rospy.Rate(1)
        # While the ROS core is running
        while not rospy.is_shutdown():
            # If have files to send and the cloud is available
            if os.listdir(path=PATH) and CLIENT.is_primary:
                # Try to send the files
                self.getFiles()
            else:
                # Wait 10 seconds for the next check
                for i in range(0,10): rate.sleep() 

# Get the files in PATH for send to cloud
    def getFiles(self):
        try:
            # Get all files in the directory
            files = sorted(os.listdir(path=PATH), reverse=True)
        except Exception as e:
            rospy.logerr("Error on get the file list\n" + e)
        for file in files:
            try:
                # Try to open the file
                get = open(file=PATH+file, mode='rb')
                # Decode to BSON
                data = bson.BSON.decode(get.read())
            except bson_errors.BSONError:
                rospy.logerr("Error on file decode\n" + e)
                get.close()
                self.rm(file=file)
            # Try send to the cloud
            if self.send2cloud(dataPath=data['dataPath'], content=data['content']):
                get.close()
                self.rm(file=file)

# Remove a file
    def rm(self, file):
        try:
            os.remove(PATH+file)
            return True
        except Exception as e:
            rospy.logerr("Error on the file remove\n" + e)
            return False

# Send the data to MongoDB cloud
    def send2cloud(self, dataPath: bson, content: bson):
        try:
            # Check if dataPath is valid
            test = dataPath['dataSource']
            test = dataPath['dataBase']
            test = dataPath['collection']
        except Exception as e:
            rospy.logerr("Error in storage data path\n" + e)
            return True
        try:
            # Try send to the cloud
            return CLIENT[dataPath['dataBase']][dataPath['collection']].insert_one(content).acknowledged
        except pymongo_erros.DuplicateKeyError:
            # If the register duplicate
            return True
        except Exception as e:
            rospy.logerr("Error when update to cloud\n" + e)
            return False


if __name__ == '__main__':
    try:
        print("Is sync?")
        bufferManager()
    except rospy.ROSInterruptException:
        pass
