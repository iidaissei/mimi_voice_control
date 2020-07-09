#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
# Title: 音声コマンドから行動を決定するプログラム
# Author: Issei Iida
# Date: 2020/02/05
#---------------------------------------------------------------------

import sys
import subprocess as sp

import rospy
import rosparam
import smach
import smach_ros
from std_msgs.msg import String
from voice_common_pkg.srv import ListenCommand, YesNo

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_function import BaseCarrier, speak
from common_action_client import *

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_voice_control/src')
from vc_common_function import *


class Listen(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes = ['listen_failed', 'motion', 'event'],
                             output_keys = ['cmd_output'])
        # Service
        self.listen_cmd_srv = rospy.ServiceProxy('/listen_command', ListenCommand)
        # Value
        self.cmd_dict = rosparam.get_param('/voice_cmd')

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN')
        result = self.listen_cmd_srv(file_name = 'voice_cmd')
        if result.result == True:
            command = result.cmd
            userdata.cmd_output = command
            state = self.cmd_dict[command]
            return state
        else:
            speak("I could't hear")
            return 'listen_failed'


class Motion(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['finish_motion'],
                             input_keys = ['cmd_input'])
        # Publisher
        self.pub_follow_req = rospy.Publisher('/chase/request', String, queue_size = 1)
        # Value
        self.bc = BaseCarrier()

    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVE')
        if userdata.cmd_input == 'turn right':
            self.bc.angleRotation(-45)
        elif userdata.cmd_input == 'turn left':
            self.bc.angleRotation(45)
        elif userdata.cmd_input == 'go straight':
            self.bc.translateDist(0.20)
        elif userdata.cmd_input == 'go back':
            self.bc.translateDist(-0.20)
        elif userdata.cmd_input == 'follow me':
            speak('start following')
            self.pub_follow_req.publish('start')
        elif userdata.cmd_input == 'stop following':
            speak('stop following')
            self.pub_follow_req.publish('stop')
        else:
            pass
        return 'finish_motion'


class Event(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['finish_event', 'finish_voice_control'],
                             input_keys = ['cmd_input'])
        # Survice
        self.listen_cmd_srv = rospy.ServiceProxy('/listen_command', ListenCommand)

    def execute(self, userdata):
        rospy.loginfo('Executing state: EVENT')
        if userdata.cmd_input == 'what is the date today':
            date = getDate()
            speak("Today is " + date)
        elif userdata.cmd_input == 'what time is it':
            time = getTime()
            speak("It's " + time)
        elif userdata.cmd_input == 'start navigation':
            speak('Where should i go?')
            location_name = self.listen_cmd_srv (file_name = 'location_name').cmd
            speak('I move to ' + location_name)
            navigationAC(coord_list)
            speak('I arrived ' + location_name)
        elif userdata.cmd_input == 'start happy time':
            speak("Welcome to Tokyo Disney Sea")
            bgmPlay('happy_time.mp3')
        elif userdata.cmd_input == 'finish voice control':
            playMessage('goodbye.wav')
            # speak('Goodbye')
            return 'finish_voice_control'
        return 'finish_event'


def main():
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    with sm_top:
        smach.StateMachine.add('LISTEN', Listen(),
                               transitions = {'listen_failed':'LISTEN',
                                              'motion':'MOTION',
                                              'event':'EVENT'},
                               remapping = {'cmd_output':'cmd_name'})

        smach.StateMachine.add('MOTION', Motion(),
                               transitions = {'finish_motion':'LISTEN'},
                               remapping = {'cmd_input':'cmd_name'})

        smach.StateMachine.add('EVENT', Event(),
                               transitions = {'finish_event':'LISTEN',
                                              'finish_voice_control':'finish_sm_top'},
                               remapping = {'cmd_input':'cmd_name'})

    outcome = sm_top.execute()


if __name__ == '__main__':
    rospy.init_node('sm_voice_control', anonymous = True)
    main()
