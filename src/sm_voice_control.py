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
                speak('Goodbye')
                return 'listen_finish'
            elif command in self.cmd_dict:
                key = self.cmd_dict[command]
                return key
        else:
            speak("I could't hear")
            return 'listen_failed'


class Motion(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_motion'],
                input_keys = ['cmd_input'])
        self.bc = BaseCarrier()

    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVE')
        if userdata.cmd_input == 'turn_right':
            speak('Rotate right')
            self.bc.angleRotation(-45)
        elif userdata.cmd_input == 'turn_left':
            speak('Rotate left')
            self.bc.angleRotation(45)
        elif userdata.cmd_input == 'go_straight':
            speak('Go forward')
            self.bc.translateDist(0.20)
        elif userdata.cmd_input == 'go_back':
            speak('Go back')
            self.bc.translateDist(-0.20)
        else:
            pass
        return 'finish_motion'


class Learn(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_learn',
                            'return_learn'],
                input_keys = ['cmd_input'])
        # Survice
        self.ggi_learning_srv = rospy.ServiceProxy('/ggi_learning', GgiLearning)
        self.location_setup_srv = rospy.ServiceProxy('/location_setup', LocationSetup)
        self.yesno_srv = rospy.ServiceProxy('/yes_no', YesNo)
        # Value
        self.file_name = ''

    def qAndA(self):
        speak('Is this corect?')
        result = self.yesno_srv()
        return result.result

    def execute(self, userdata):
        rospy.loginfo('Executing state: LEARN')
        if userdata.cmd_input == 'add_location':
            speak('Please tell me the location name')
            location_name = 'shelf'# 場所名サーバー
            speak('Location is '+ location_name)
            if self.qAndA():
                # self.location_setup_srv(state = 'add', name = location_name)
                speak('Location added')
                return 'finish_learn'
            else:
                speak("Oops!")
                return 'return_learn'
        elif userdata.cmd_input == 'save_location':
            speak('Please tell me the file name')
            #名前聞くサービス
            # self.location_setup_srv(state = 'save', name = file_name)
            self.file_name = 'gpsr'
            speak('File name is ' + self.file_name)
            if self.qAndA():
                speak('Location saved')
                return 'finish_learn'
            else:
                speak('Oops!')
                return 'return_learn'
        elif userdata.cmd_input == 'save_map':
            speak('Save the map as ' + self.file_name)
            sp.Popen(['rosrun','map_server','map_saver','-f','/home/athome/map/'+ self.file_name])
            rospy.sleep(1.0)
            speak('Map saved')
        elif userdata.cmd_input == 'start_ggi_learning':
            #self.ggi_learning_srv()
            speak('start ggi learning')
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
            self.pub_follow_req.publish('start')
            speak('I will follow you')
        elif userdata.cmd_input == 'stop_follow':
            self.pub_follow_req.publish('stop')
            speak('Stop following')
        elif userdata.cmd_input == 'start_map_creating':
            speak('Start map creating')
            rospy.sleep(0.1)
            sp.Popen(['roslaunch','turtlebot_navigation','gmapping_demo.launch'])
            rospy.sleep(0.5)
            # sp.Popen(['roslaunch','turtlebot_teleop','keyboard_teleop.launch'])
            rospy.sleep(0.5)
            sp.Popen(['roslaunch','turtlebot_rviz_launchers','view_navigation.launch'])
            
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
                transitions = {'finish_learn':'LISTEN',
                               'return_learn':'LEARN'},
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
