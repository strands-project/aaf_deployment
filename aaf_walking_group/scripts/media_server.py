#!/usr/bin/env python

import rospy
from mongodb_media_server import MediaClient
from aaf_walking_group.srv import GetMediaId, GetMediaIdRequest, GetMediaIdResponse



class MediaServer(object):
    def __init__(self, name):
        rospy.loginfo("Creating "+name+" server")
        self.media_set = rospy.get_param('~media_set')
        self.media_index = 0

        self.mc = MediaClient(
            rospy.get_param("mongodb_host"),
            rospy.get_param("mongodb_port")
        )

        sets = self.mc.get_sets(rospy.get_param("~media_set_type"))
        object_id = None
        for s in sets:
            if s[0] == self.media_set:
                object_id = s[2]

        if object_id is None:
            rospy.logwarn('Could not find any set in database matching %s' % self.media_set)
            return

        self.file_names = self.mc.get_set(object_id)

        if len(self.file_names) == 0:
            rospy.logwarn('No files available in set %s' % self.media_set)

        for f in self.file_names:
            print "Media name:", f[0]

        self.media_srv = rospy.Service('/aaf_walking_group'+name+'/get_id', GetMediaId, self.get_id)

        rospy.loginfo(" ... started "+name)

    def get_id(self, req):
        if req.action == GetMediaIdRequest.CURRENT:
            return GetMediaIdResponse(str(self.file_names[self.media_index][0]))
        elif req.action == GetMediaIdRequest.NEXT:
            self.media_index = self.media_index + 1
            if self.media_index >= len(self.file_names):
                self.media_index = 0
            return GetMediaIdResponse(str(self.file_names[self.media_index][0]))
        elif req.action == GetMediaIdRequest.PREVIOUS:
            self.media_index = self.media_index - 1
            if self.media_index < 0:
                self.media_index = len(self.file_names) - 1
            return GetMediaIdResponse(str(self.file_names[self.media_index][0]))
        else:
            raise rospy.ServiceException("Unknow action")


if __name__ == '__main__':
    rospy.init_node('media_server')
    server = MediaServer(rospy.get_name())
    rospy.spin()
