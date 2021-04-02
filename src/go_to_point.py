#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf import transformations
import math
from goal_publisher.msg import PointArray
from std_srvs.srv import *

import math

active_ = False
goal_ =PointArray()
initial_position_ = Point()
initial_position_.x = 0.001
initial_position_.y = 0.001
initial_position_.z = 0
desired_position_ = Point()
desired_position_.x =0
desired_position_.y =0
desired_position_.z = 0
distance_diff = 0.5
goal_1_comp=False
goal_2_comp=False
goal_3_comp=False
# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
#desired_position_ = Point()
#desired_position_.x = rospy.get_param('des_pos_x')
#desired_position_.y = rospy.get_param('des_pos_y')
#desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.3

# publishers
pub = None

# service callbacks
def go_to_point_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def clbk_point(data):
    global desired_position_,goal_1_comp,goal_2_comp,goal_3_comp,goal_
    goal_1_x = data.goals[0].x
    goal_1_y = data.goals[0].y
    goal_2_x = data.goals[1].x
    goal_2_y = data.goals[1].y
    goal_3_x = data.goals[2].x
    goal_3_y = data.goals[2].y


    if(abs(goal_1_x - position_.x)>distance_diff and abs(goal_1_y - position_.y)>distance_diff and goal_1_comp==False):
        #goal_1_comp=False
        print("Towards Goal 1")
        desired_position_.x = goal_1_x
        desired_position_.y = goal_1_y
        initial_position_.x = position_.x
        initial_position_.y = position_.y
    elif(abs(goal_1_x - position_.x)<distance_diff and abs(goal_1_y - position_.y)<distance_diff):
        goal_1_comp=True
        print("Goal 1 Completed")
        desired_position_.x = goal_2_x
        desired_position_.y = goal_2_y
        initial_position_.x = position_.x
        initial_position_.y = position_.y
    elif(abs(goal_2_x - position_.x)<distance_diff and abs(goal_2_y - position_.y)<distance_diff):
        goal_2_comp=True
        print("Goal 2 Completed")
        desired_position_.x = goal_3_x
        desired_position_.y = goal_3_y
        initial_position_.x = position_.x
        initial_position_.y = position_.y
    elif(abs(goal_3_x - position_.x)<distance_diff and abs(goal_3_y - position_.y)<distance_diff):
        goal_3_comp=True
        print("Goal 3 Completed")
    else:
        print("Going towards the goal")

# callbacks
def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose[1].position

    # yaw
    quaternion = (
        msg.pose[1].orientation.x,
        msg.pose[1].orientation.y,
        msg.pose[1].orientation.z,
        msg.pose[1].orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def change_state(state):
    global state_
    state_ = state
    print 'State changed to [%s]' % state_

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)

    rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(1)

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.6
        twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
        pub.publish(twist_msg)
    else:
        print 'Position error: [%s]' % err_pos
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(0)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def main():
    global pub, active_

    rospy.init_node('go_to_point')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_odom = rospy.Subscriber('/gazebo/model_states', ModelStates, clbk_odom)

    sub_go = rospy.Subscriber('/goals', PointArray, clbk_point)

    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                done()
            else:
                rospy.logerr('Unknown state!')

        rate.sleep()

if __name__ == '__main__':
    main()
