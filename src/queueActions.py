#!/usr/bin/env python3


# Import config
from config import *

# Import librarys
import pymongo, bson, datetime, rospy, os, json, subprocess, fractions
from pymongo import errors as pymongo_erros

# Messages
from std_msgs.msg import String

# Define default action collection
COLL = '/actions'

# Queue action class
class queueActions:

    def __init__(self) -> None:
        # Start the node
        rospy.init_node('queueActions', anonymous=False)
        rospy.loginfo("Queue of actions topics started")
        # Create a subscriber to get action from another ROS topics
        rospy.Subscriber("addNewActions", String, self.callback_addNewActions)
        rospy.loginfo("Create the subscriber /addNewActions")
        # Define a global queue empty
        self.LocalQueue = []
        # Do the initial query
        rospy.loginfo("Starting synchronisation")
        while not self.initialQuery():
            rospy.sleep(5)
            None
        # Awaiting instructions
        rospy.loginfo("Awaiting actions")
        while not rospy.is_shutdown():
            # Get action from cloud
            self.getCloudActions()
            # Try to execute the actions in the local queue
            self.runLocalQueue()
            # Sync with cloud
            self.send2cloud(content=self.LocalQueue)
            # Wait for 1 second
            rospy.sleep(1)

    # Get the list of action available on cloud, or run a pipeline
    def queryCloud(self, pipeline=None):
        # Get the list from cloud
        try:
            result = list(CLIENT[DATALAKE][COLL].aggregate(pipeline=pipeline))
            return result
        except Exception as e:
            rospy.logerr("Error on query the cloud")
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return None
    
    # Local action send callback
    def callback_addNewActions(self, msg):
        # Get the value of msg
        try:
            data = msg.data
            data = json.loads(data)
        except Exception as e:
            rospy.logerr("Error on the convert the mesage: " + str(msg))
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
        # Get the values of command and priority
        try:
            _data = {
                "_id": bson.ObjectId(),
                "command": data['command'],
                "dateTime": datetime.datetime.now(),
                "source": 'robot',
                "status": 'wait'
            }
        except Exception as e:
            rospy.logerr("Error on parse a new action")
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
        # Add the action to the local queue
        try: 
            self.add2LocalQueue(_data)
        except Exception as e:
            rospy.logerr("Error on add to local queue")
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)

    # Run action
    def runAction(self, action):
        # Try to execute the action
        try:
            _command = action['command']
            rospy.loginfo("Run command " + _command)
            result = subprocess.Popen(_command, shell=True)
            rospy.loginfo("Send")
            action['status'] = 'runned'
        except Exception as e:
            rospy.logerr("Error in the execution of " + str(action))
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            action['status'] = 'error'
            
    # Add action to the queue
    def add2LocalQueue(self, data):
        # Check local queue
        try:
            rospy.loginfo("Add action")
            for action in self.LocalQueue:
                if data['source'] == 'admin':
                    action['status'] = 'canceled'
                # Action is not on list and is wait
                elif action['command'] == data['command'] and action['status'] == 'wait':
                    data['status'] = 'canceled'
        except Exception as e:
            rospy.logerr("Error in the check local queue for the action: " + str(data))
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
        # Add the action to the local queue
        try:
            self.LocalQueue.append(data)
        except Exception as e:
            rospy.logerr("Error add the action: " + str(data))
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
    
    # Add|update documents to the cloud
    def send2cloud(self, content) -> bool:
        # Check if content is a list or just one document
        if not isinstance(content, list):
            content = [content]
        # Send or uupdate all documents in content
        for document in content:
            # Try send to the cloud
            try:
                _result = CLIENT[DATALAKE][COLL].find_one_and_update(filter={'_id': document['_id']},update={'$set':document}, upsert=True)
                # If document is not in 'wait' delete
                if not _result['status'] == 'wait':                    
                    content.remove(document)
                    rospy.rospy.loginfo("Remove action from queue: " + str(document))
            except Exception as e:
                rospy.logerr("Error on send document to cloud: " + str(document) )
                rospy.logerr("An exception occurred:", type(e).__name__,e.args)
                return False
        return True

    # The initial query
    def initialQuery(self) -> bool:
        PIPELINE_CHANGE_WAIT = [
            {
                '$match': {
                    'status': 'wait'
                }
            }, {
                '$addFields': {
                    'status': 'canceled'
                }
            }
        ]
        rospy.loginfo("Run the initial query againts " + COLL)
        # Get 'wait' actions in cloud
        initial = self.queryCloud(pipeline=PIPELINE_CHANGE_WAIT)
        # Send documents to cloud
        return self.send2cloud(initial)

    # Get actions from cloud
    def getCloudActions(self):
        # Get just wait items aggregations
        PIPELINE_GET_WAIT = [{
            '$match': {
                'status': 'wait'
            }
        },]
        _result = self.queryCloud(pipeline=PIPELINE_GET_WAIT)
        # Add actions to the local queu
        for action in _result:
            self.add2LocalQueue(action)
        
    # Run the actions in the local queue
    def runLocalQueue(self):
        # Run all actions in localqueue
        for action in self.LocalQueue:
            try:
                # Check if the action is waiting
                if action['status'] == 'wait':
                    self.runAction(action=action)
            except Exception as e:
                rospy.logerr("Error action: " + str(action) )
                rospy.logerr("An exception occurred:", type(e).__name__,e.args)


if __name__ == '__main__':
    try:
        # Run action
        queueActions()
    except rospy.ROSInterruptException:
        pass
