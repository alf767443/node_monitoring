#!/usr/bin/env python3

# Import topics
from topics import TOPICS

# Import config
from config import *

# Import librarys
import rospy, bson, os, genpy, string, random
from pymongo import errors as pymongo_erros
from fractions import Fraction
from datetime import datetime

# Listen topic class
class listenTopics:
    def __init__(self, TOPICS) -> None:
        # Starts unique node in the ROS core with the name listenNode
        rospy.init_node('listenTopics', anonymous=False)
        rospy.loginfo("Listen ROS topics started")
        # Reads out the list of topics present in the file topics.py
        self.TOPICS = self.fillNode(topics=TOPICS)
        # Set up the subscribers for each item in TOPICS
        for topic in self.TOPICS:
            try:
                # Creates the subscriber
                self.newSubscriber(topic=topic)
            except Exception as e:
                rospy.logerr("Error in topic.py in the topic" + topic['topic'])
                rospy.logerr("An exception occurred:", type(e).__name__,e.args)
        # Keeps the topic active
        rospy.spin()

# Create a random string
    def randomString(self, size):
        chars = string.ascii_letters + string.digits
        return ''.join(random.choice(chars) for _ in range(size))


# Fill Nodes default info 
    def fillNode(self, topics):
        # Deafult values
        def defaultValues(topic):
            # Create the default values dictionary
            try:
                _return = {
                    'sleep'    : 1,                     # Sleep of 1 second
                    'callback': None,                   # No callback function
                    'dataPath': {
                        'dataSource': DATASOURCE,       # Add DATASOURCE of config.py
                        'dataBase'  : DATALAKE,         # Add DATALAKE of config.py
                        'collection': topic['topic']      # Collection name is the same of of topic name
                    }
                }
                return _return
            except Exception as e:
                rospy.logerr("Error in default values")
                rospy.logerr("An exception occurred:", type(e).__name__,e.args)
                return None
        # Update the dictionary
        def update_nested_dict(default, target):
            # Without default values
            if default == None:
                rospy.logerr("No default dictionary for: ", target)
                target = None
            # Run for all dictionary
            for key, value in default.items():
                try:
                    # Check if nested
                    if isinstance(value, dict) and key in target and isinstance(target[key], dict):
                        update_nested_dict(value, target[key])
                    else:
                        target.setdefault(key, value)
                except Exception as e:
                    rospy.logerr("Error in the key:", key, value)
                    rospy.logerr("An exception occurred:", type(e).__name__,e.args)
        # Copy Nodes
        try:
            _nodes = topics.copy()
        except Exception as e:
            rospy.logerr("Error in copy of TOPICS")
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
        # Run for all codes        
        for topic in _nodes:
            try:
                update_nested_dict(defaultValues(topic=topic), topic)
            except Exception as e:
                rospy.logerr("Error in set values of:", topic)
                rospy.logerr("An exception occurred:", type(e).__name__,e.args)
        return _nodes
               
# Create new subscriber
    def newSubscriber(self, topic): 
        try:    
            # Uses the information in the topic dictionary to create a subscriber
            rospy.Subscriber(name=topic['topic'], data_class=topic['msg'], callback=self.callback, callback_args=topic, queue_size=1)

            rospy.loginfo("Subscriber to the topic " + topic['topic'] + " create")
            return True
        except Exception as e:
            rospy.logerr("Error in the creation of subscriber in the topic" + topic['topic'])
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return False
        
# Callback to the topic
    def callback(self, msg, args):
        try:
            # Gets the message data
            data = self.msg2document(msg=msg)
            # Adds the date 
            data.update({'dateTime': datetime.now(), 'robot': ROBOT_NAME})
        except Exception as e:
            rospy.logerr("Error to convert the mensage in the topic" + args['topic'])
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
        try:
            # If the topic has a callback function it executes
            if args['callback'] != None:
                # Execute the callback function
                args['callback'](data=data, topic=args)
        except Exception as e:
            rospy.logerr("Error in callback function in the topic" + args['topic'])
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
        if isinstance(data, (dict, list)) and data != {}:
            try:
                if CLIENT.is_primary:
                    if not self.send2cloud(dataPath=args['dataPath'], content=data):
                        # If can't send, create a file
                        self.createFile(dataPath=args['dataPath'], content=data)     
                else:
                    self.createFile(dataPath=args['dataPath'], content=data)     
            except (pymongo_erros.ConnectionFailure, pymongo_erros.ServerSelectionTimeoutError):
                # Create the storage file
                self.createFile(dataPath=args['dataPath'], content=data)
            except Exception as e:
                rospy.logerr("Error with MongoDB client")
                rospy.logerr("An exception occurred:", type(e).__name__,e.args)       
        # Wait the set time
        rospy.sleep(args['sleep'])
    
# Create a file that contains the information for storage
    def createFile(self, dataPath: bson, content: bson):
        try:
            # Check if dataPath is valid
            test = dataPath['dataSource']
            test = dataPath['dataBase']
            test = dataPath['collection']
        except Exception as e:
            rospy.logerr("Error in storage data path")
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return False
        try:
            # Create data string
            data = bson.encode(document={'dataPath': dataPath, 'content': content})
            # Create the file name
            _dateTime_now = datetime.strftime(datetime.now(),"%Y%m%d%H%M%S_%f")
            fileName =  f"{dataPath['collection'].replace('/', '_')}_{_dateTime_now}_{self.randomString(size=10)}"
            # Define the extension
            extencion = '.cjson'
            # Create the extension
            fullPath = PATH+fileName+extencion
            # Create directory if it don't exist
            if not os.path.exists(path=PATH):
                os.chmod
                os.makedirs(name=PATH)
            # Create file
            file = open(file=fullPath, mode='wb')
            # Fill file
            file.write(data)
            file.close()
            return True
        except Exception as e:
            rospy.logerr("Error to create the file")
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return False

# Send the data to MongoDB cloud
    def send2cloud(self, dataPath: bson, content: bson):
        try:
            # Check if dataPath is valid
            test = dataPath['dataSource']
            test = dataPath['dataBase']
            test = dataPath['collection']
        except Exception as e:
            rospy.logerr("Error in storage data path")
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return True
        try:
            # Try send to the cloud
            if not isinstance(content, list):
                content = [content]
            return CLIENT[dataPath['dataBase']][dataPath['collection']].insert_many(content).acknowledged
        except pymongo_erros.DocumentTooLarge:
            rospy.logwarn(f"The message from {dataPath['collection']} is too large to store.")
            return True
        except pymongo_erros.DuplicateKeyError:
            # If the register duplicate
            return True
        except Exception as e:
            rospy.logerr("Error when update to cloud")
            rospy.logerr("An exception occurred:", type(e).__name__,e.args)
            return False
        
# Convert message to a python document
    def msg2document(self, msg):
        d = {}
        slot_types = []
        if hasattr(msg,'_slot_types'):
            slot_types = msg._slot_types
        else:
            slot_types = [None] * len(msg.__slots__)
        for (attr, type) in zip(msg.__slots__, slot_types):
            d[attr] = self.sanitize_value(attr, getattr(msg, attr), type)
        return d
    def sanitize_value(self, attr, v, type):
        if isinstance(v, str):
            if type == 'uint8[]':
                v = Binary(v)
            else:
                try:
                    v = str(v)
                except UnicodeDecodeError as e:
                    v = Binary(v)
            return v
        if isinstance(v, rospy.Message):
            return self.msg2document(v)
        elif isinstance(v, genpy.rostime.Time):
            return self.msg2document(v)
        elif isinstance(v, genpy.rostime.Duration):
            return self.msg2document(v)
        elif isinstance(v, list):
            result = []
            for t in v:
                if hasattr(t, '_type'):
                    result.append(self.sanitize_value(None, t, t._type))
                else:
                    result.append(self.sanitize_value(None, t, None))
            return result
        else:
            return v

if __name__ == '__main__':
    try:
        listenTopics(TOPICS=TOPICS)
    except rospy.ROSInterruptException:
        pass

