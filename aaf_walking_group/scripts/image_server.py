#!/usr/bin/env python

import rospy
import actionlib
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from mongodb_media_server import MediaClient
from aaf_walking_group.msg import EmptyAction
from std_srvs.srv import Empty, EmptyResponse

from os.path import join, exists, expanduser
from os import makedirs


class ImageServer(object):
    def __init__(self, name):
        rospy.loginfo("Creating "+name+" server")
        self._as = actionlib.SimpleActionServer(
            '/aaf_walking_group'+name,
            EmptyAction,
            self.execute,
            False
        )
        self.image_folder = join(expanduser('~'), '.ros', 'image_server')
        self.image_set = rospy.get_param('~image_set', 'walking_group_pictures')
        self.img_pub = rospy.Publisher(
            rospy.get_param("~image_pub", "/aaf_walking_group"+name+"/image"),
            Image,
            queue_size=2
        )
        self.image_index = 0
        self.bridge = CvBridge()

        self.mc = MediaClient(
            rospy.get_param("mongodb_host"),
            rospy.get_param("mongodb_port")
        )

        sets = self.mc.get_sets("Photo")
        object_id = None
        for s in sets:
            if s[0] == self.image_set:
                object_id = s[2]

        if object_id is None:
            rospy.logwarn('Could not find any set in database matching image_set')
            return

        file_set = self.mc.get_set(object_id)

        if len(file_set) == 0:
            rospy.logwarn('No images available in set ' + self.image_set)

        for f in file_set:
            print "Media name:", f[0]

        if not exists(self.image_folder):
            makedirs(self.image_folder)

        for f in file_set:
            file = self.mc.get_media(str(f[2]))
            outfile = open(join(self.image_folder, f[0]), 'wb')
            filestr = file.read()
            outfile.write(filestr)
            outfile.close()

        self.file_names = [join(self.image_folder, f[0]) for f in file_set]

        self.next_srv = rospy.Service('/aaf_walking_group'+name+'/next', Empty, self.next_img)
        self.prev_srv = rospy.Service('/aaf_walking_group'+name+'/prev', Empty, self.prev_img)

        rospy.loginfo(" ... starting "+name)
        self._as.start()
        rospy.loginfo(" ... started "+name)

        self.cv_image = self.bridge.cv2_to_imgmsg(cv2.imread(self.file_names[self.image_index]), "bgr8")

    def execute(self, goal):
        rate = rospy.Rate(5) # mjpeg_server does not work with latched topics :(
        # Exhaustive experimental research has proven that 5 is the best magic number there is.
        # Same CPU usage as 4, but mjpeg has a lower delay and feels much snappier.
        # Never go to 10. NEVER!
        while not rospy.is_shutdown() and not self._as.is_preempt_requested():
            if self.img_pub.get_num_connections():
                self.pub_image(self.cv_image)
            rate.sleep()
        if not self._as.is_preempt_requested():
            self._as.set_succeeded()
        else:
            self._as.set_preempted()

    def pub_image(self, image):
        self.img_pub.publish(image)

    def next_img(self, req):
        self.image_index = self.image_index + 1
        if self.image_index >= len(self.file_names):
            self.image_index = 0
        print self.file_names[self.image_index]
        self.cv_image = self.bridge.cv2_to_imgmsg(cv2.imread(self.file_names[self.image_index]), "bgr8")
        return EmptyResponse()

    def prev_img(self, req):
        self.image_index = self.image_index - 1
        if self.image_index < 0:
            self.image_index = len(self.file_names) - 1
        print self.file_names[self.image_index]
        self.cv_image = self.bridge.cv2_to_imgmsg(cv2.imread(self.file_names[self.image_index]), "bgr8")
        return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node('image_server')
    server = ImageServer(rospy.get_name())
    rospy.spin()
