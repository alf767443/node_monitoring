#!/usr/bin/env python3

# Import nodes
from nodes import NODES

# Import config
from config import PATH, CLIENT

# Import librarys
import rospy, bson, os, genpy
from pymongo import errors as pymongo_erros
from fractions import Fraction
from datetime import datetime

class listenNodes:
    def __init__(self, NODES) -> None:
        # Starts unique node in the ROS core with the name listenNode
        rospy.init_node('listenNodes', anonymous=False)
        rospy.loginfo("Listen ROS nodes started")
        # Reads out the list of nodes present in the file nodes.py
        self.NODES = NODES
        # Set up the subscribers for each item in NODES
        for node in self.NODES:
            try:
                # Set node rate and ticks
                self.sleepDef(node=node)
                # Creates the subscriber
                self.newSubscriber(node=node)
            except Exception as e:
                rospy.logerr("Error in node.py in the node" + node['node'])
                rospy.logerr(e)
        # Keeps the node active
        rospy.spin()
               
# Create new subscriber
    def newSubscriber(self, node): 
        try:
            # Uses the information in the node dictionary to create a subscriber
            rospy.Subscriber(name='/' + node['node'], data_class=node['msg'], callback=self.callback, callback_args=node, queue_size=1)

            rospy.loginfo("Subscriber to the node /" + node['node'] + " create")
            return True
        except Exception as e:
            rospy.logerr("Error in the creation of subscriber in the node" + node['node'])
            rospy.logerr(e)
            return False
        
# Callback to the node
    def callback(self, msg, args):
        try:
            # Gets the message data
            data = self.msg2document(msg=msg)
            # Adds the date 
            data.update({'dateTime': datetime.now()})
        except Exception as e:
            rospy.logerr("Error to convert the mensage in the node" + args['node'])
            rospy.logerr(e)
        try:
            # If the node has a callback function it executes
            if args['callback'] != None:
                # Execute the callback function
                args['callback'](data)
        except Exception as e:
            rospy.logerr("Error in callback function in the node" + args['node'])
            rospy.logerr(e)
        if isinstance(data, (dict, list)):
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
                rospy.logerr(e)
            
        # Wait the set time
        for i in range(0,args['ticks']): args['rate'].sleep()

# Set the rate and ticks parameters for the node
    def sleepDef(self, node):
        try: 
            # Find the frequency for the node and the number of sleep ticks
            fraction = Fraction(node['rate']).limit_denominator()
            rate = fraction.numerator
            ticks = fraction.denominator
            # Define this data node
            node['rate'] = rospy.Rate(rate)
            node['ticks'] = ticks
            rospy.logdebug("\n\tNode: " + node['node'] + "\n\trate: " + str(rate) + "\n\tticks: " + str(ticks))
            return True
        except Exception as e:
            rospy.logerr("Error to create the timer")
            rospy.logerr(e)
            return False
    
# Create a file that contains the information for storage
    def createFile(self, dataPath: bson, content: bson):
        try:
            # Check if dataPath is valid
            test = dataPath['dataSource']
            test = dataPath['dataBase']
            test = dataPath['collection']
        except Exception as e:
            rospy.logerr("Error in storage data path")
            rospy.logerr(e)
            return False
        try:
            # Create data string
            data = bson.encode(document={'dataPath': dataPath, 'content': content})
            # Create the file name
            fileName =  datetime.strftime(datetime.now(),"%Y%m%d%H%M%S_%f")
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
            rospy.logerr(e)
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
            rospy.logerr(e)
            return True
        try:
            # Try send to the cloud
            if not isinstance(content, list):
                content = [content]
            return CLIENT[dataPath['dataBase']][dataPath['collection']].insert_many(content).acknowledged
        except pymongo_erros.DuplicateKeyError:
            # If the register duplicate
            return True
        except Exception as e:
            rospy.logerr("Error when update to cloud")
            rospy.logerr(e)
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
        listenNodes(NODES=NODES)
    except rospy.ROSInterruptException:
        pass
