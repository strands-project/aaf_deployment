#!/usr/bin/env python

import rospy
from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTasks, SetExecutionStatus
from strands_navigation_msgs.msg import *
import sys
from geometry_msgs.msg import PoseStamped
from mongodb_store.message_store import MessageStoreProxy

def get_services():
    # get services necessary to do the jon
    add_tasks_srv_name = '/task_executor/add_tasks'
    set_exe_stat_srv_name = '/task_executor/set_execution_status'
    rospy.loginfo("Waiting for task_executor service...")
    rospy.wait_for_service(add_tasks_srv_name)
    rospy.wait_for_service(set_exe_stat_srv_name)
    rospy.loginfo("Done")        
    add_tasks_srv = rospy.ServiceProxy(add_tasks_srv_name, AddTasks)
    set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
    return add_tasks_srv, set_execution_status



if __name__ == '__main__':
    rospy.init_node("example_multi_add_client")

    # get services to call into execution framework
    add_tasks, set_execution_status = get_services()
    msg_store = MessageStoreProxy() 


    task=Task()
    task.action='/go_to_person_action'
    duration = 5 * 60
    task.max_duration = rospy.Duration(duration)
    object_id = msg_store.insert(PoseStamped())
    task_utils.add_object_id_argument(task, object_id, PoseStamped)
    task_utils.add_bool_argument(task, False)
    task_utils.add_float_argument(task, duration)
  
    add_tasks([task])
    set_execution_status(True)
  


