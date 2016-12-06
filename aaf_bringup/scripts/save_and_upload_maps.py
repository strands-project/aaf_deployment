#!/usr/bin/env python
# license removed for brevity

import os
import yaml
from datetime import datetime
import rospy
from std_msgs.msg import String
#from std_msgs.msg import Int64
from nav_msgs.msg import OccupancyGrid
#from mongodb_store.message_store import MessageStoreProxy


class predicted_map_saver(object):

    def __init__(self) :
        self.first_charge=True
        self.visited_nodes=[]
        rospy.on_shutdown(self._on_node_shutdown)
        rospy.Subscriber('/current_node', String, self.current_node_callback)
        rospy.Subscriber('/gmap', OccupancyGrid, self.gmap_callback)
        self.folder_name=rospy.get_param('~map_folder','/storage/predicted_maps/')
        rospy.spin()

    def gmap_callback(self, msg):
        self.route_map = msg
        
        
    def current_node_callback(self, msg):
        print "current node: ", msg
        if msg.data != 'none':
            self.visited_nodes.append(msg.data)
        if msg.data == 'ChargingPoint' and not self.first_charge:
            self.save_pred_map()
        else:
            if msg.data != 'ChargingPoint':
                self.first_charge = False

    
    def save_pred_map(self):
        self.timestamp = int(rospy.get_time())
        print "Getting map for: ", self.timestamp

        self.map_name = rospy.get_param('topological_map_name', 'world')
        print "got map: ",self.map_name
        self.upload_map()
        rospy.sleep(2.0)
        os.system('rosnode kill /slam_gmapping')
        self.visited_nodes=[]


    def upload_map(self):

        meta = {}
        meta["epoch"] = self.timestamp
        meta["date"] = datetime.fromtimestamp(self.timestamp).strftime('%A, %B %d %Y, at %H:%M:%S hours')
        meta["pointset"] = self.map_name
        meta["route"] = self.visited_nodes
        filename = self.folder_name+str(self.timestamp)
        meta["filename"] = filename

        filename2=filename+'.meta'
        yml = yaml.safe_dump(meta, default_flow_style=False)
        print filename, filename2
               
        fh = open(filename2, "w")
        s_output = str(yml)
        print s_output
        fh.write(s_output)
        fh.close            


        rospy.sleep(2.0)
        copycmdstr = 'rosrun map_server map_saver -f '+filename+' map:=gmap'
        print copycmdstr
        os.system(copycmdstr)


    def _on_node_shutdown(self):
        os.system('rosnode kill /slam_gmapping')
        print "BYE"

if __name__ == '__main__':
    rospy.init_node('predicted_map_saver')
    server = predicted_map_saver()