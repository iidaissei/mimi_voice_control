#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
# Title: 音声コマンドから行動を決定するプログラム
# Author: Issei Iida
# Date: 2020/02/05
# Memo:
#---------------------------------------------------------------------

# Python
import sys
import subprocess as sp
# ROS
import rospy
import rosparam
import smach
import smach_ros
from std_msgs.msg import String
from ggi.srv import ListenCommand, GgiLearning, YesNo
from mimi_common_pkg.srv import LocationSetup

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_function import BaseCarrier, speak


class Listen(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['listen_failed',
                            'motion',
                            'learn',
                            'event',
                            'listen_finish'],
                output_keys = ['cmd_output'])
        # Service
        self.listen_cmd_srv = rospy.ServiceProxy('/listen_command', ListenCommand)
        # Value
        self.cmd_dict = rosparam.get_param('/voice_cmd')

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN')
        result = self.listen_cmd_srv()
        if result.result == True:
            command = result.cmd
            userdata.cmd_output = command
            rospy.loginfo('Command is *' + str(command) + '*')
            if command == 'finish_voice_control':
                speak('Good night')
                return 'listen_finish'
            if command in self.cmd_dict:
                key = self.cmd_dict[command]
                return key
            else:
                rospy.loginfo(str(command) + ' is not found')
                return 'listen_failed'
        else:
            speak('One more time pleas')
            rospy.loginfo('Listening failed')
            return 'listen_failed'


class Motion(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_motion'],
                input_keys = ['cmd_input'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVE')
        if userdata.cmd_input == 'turn_right':
            rospy.loginfo('Turn right')
            speak('Rotate right')
            # self.bc.angleRotation(-45)
        elif userdata.cmd_input == 'turn_left':
            rospy.loginfo('Turn left')
            speak('Rotate left')
            # self.bc.angleRotation(45)
        elif userdata.cmd_input == 'go_straight':
            rospy.loginfo('Go straight')
            speak('Go forward')
            # self.bc.translateDist(0.20)
        elif userdata.cmd_input == 'go_back':
            rospy.loginfo('Go back')
            speak('Go back')
            # self.bc.translateDist(-0.20)
        else:
            pass
        return 'finish_motion'


class Learn(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_learn'],
                input_keys = ['cmd_input'])
        # Survice
        self.ggi_learning_srv = rospy.ServiceProxy('/ggi_learning', GgiLearning)
        self.location_setup_srv = rospy.ServiceProxy('/location_setup', LocationSetup)
        self.yesno_srv = rospy.ServiceProxy('/yes_no', YesNo)

    def execute(self, userdata):
        rospy.loginfo('Executing state: LEARN')
        if userdata.cmd_input == 'add_location':
            speak('Please tell me location name')
            # 場所名サーバー起動
            # location_name = self.ggi_learning_srv()
            # self.location_setup_srv(state = 'add', name = location_name)
            speak('Complete adding name')
        elif userdata.cmd_input == 'save_location':
            speak('Please tell me file name')
            #名前聞くサービス
            # self.location_setup_srv(state = 'save', name = file_name)
            speak('Do you saving map?')
            result = self.yesno_srv()
            if result.result == True:
                speak('Save map')
                # sp.Popen(['rosrun','map_server','map_sarver','-f','~/map/'+ file_name])
                rospy.sleep(1.0)
                speak('Save complete')
            else:
                speak('Complete save location')
                pass
        elif userdata.cmd_input == 'start_learning':
            self.ggi_learning_srv()
        return 'finish_learn'


class Event(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_event'],
                input_keys = ['cmd_input'])
        # Publisher
        self.pub_follow_req = rospy.Publisher('/chase/request', String, queue_size = 1)

    def execute(self, userdata):
        rospy.loginfo('Executing state: FOLLOW')
        if userdata.cmd_input == 'start_follow':
            # self.pub_follow_req.publish('start')
            speak('I will follow you')
        elif userdata.cmd_input == 'stop_follow':
            # self.pub_follow_req.publish('stop')
            speak('Stop following')
        elif userdata.cmd_input == 'start_map_creating':
            speak('Start map creating')
            sp.Popen(['roslaunch','turtlebot_navigation','gmapping_demo.launch'])
            rospy.sleep(1.0)
            sp.Popen(['roslaunch','turtlebot_rviz_launchers','view_navigation.launch'])
            rospy.sleep(1.0)
            sp.Popen(['roslaunch','turtlebot_teleop','keyboard_teleop.launch'])
            rospy.sleep(1.0)
        return 'finish_event'


def main():
    sm_top = smach.StateMachine(outcomes = ['finish_voice_control'])

    with sm_top:
        smach.StateMachine.add(
                'LISTEN',
                Listen(),
                transitions = {'listen_failed':'LISTEN',
                               'motion':'MOTION',
                               'learn':'LEARN',
                               'event':'EVENT',
                               'listen_finish':'finish_voice_control'},
                remapping = {'cmd_output':'cmd_name'})

        smach.StateMachine.add(
                'MOTION',
                Motion(),
                transitions = {'finish_motion':'LISTEN'},
                remapping = {'cmd_input':'cmd_name'})

        smach.StateMachine.add(
                'LEARN',
                Learn(),
                transitions = {'finish_learn':'LISTEN'},
                remapping = {'cmd_input':'cmd_name'})

        smach.StateMachine.add(
                'EVENT',
                Event(),
                transitions = {'finish_event':'LISTEN'},
                remapping = {'cmd_input':'cmd_name'})

    outcome = sm_top.execute()


if __name__ == '__main__':
    rospy.init_node('sm_voice_control', anonymous = True)
    main()
