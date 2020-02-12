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
from gpsr.srv import ActionPlan
from ggi.srv import ListenCommand, GgiLearning, YesNo
from mimi_common_pkg.srv import LocationSetup

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_function import BaseCarrier, speak
from common_action_client import *

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_voice_control/src')
from vc_common_function import *


class Listen(smach.State): 
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['listen_failed',
                            'motion',
                            'learn',
                            'event',
                            'stask'],
                output_keys = ['cmd_output',
                               'words_output'])
        # Service
        self.listen_cmd_srv = rospy.ServiceProxy('/listen_command', ListenCommand)
        # Value
        self.cmd_dict = rosparam.get_param('/voice_cmd')
        self.words_dict = rosparam.get_param('/first_words')

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN')
        result = self.listen_cmd_srv()
        if result.result == True:
            command = result.cmd
            userdata.cmd_output = command
            userdata.words_output = self.words_dict[command]
            state = self.cmd_dict[command]
            return state
        else:
            speak("I could't hear")
            return 'listen_failed'


class Motion(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_motion'],
                input_keys = ['cmd_input', 'words_input'])
        self.bc = BaseCarrier()

    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVE')
        speak(userdata.words_input)
        if userdata.cmd_input == 'turn_right':
            self.bc.angleRotation(-45)
        elif userdata.cmd_input == 'turn_left':
            self.bc.angleRotation(45)
        elif userdata.cmd_input == 'go_straight':
            self.bc.translateDist(0.20)
        elif userdata.cmd_input == 'go_back':
            self.bc.translateDist(-0.20)
        else:
            pass
        return 'finish_motion'


class STask(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_stask', 'finish_voice_control'],
                input_keys = ['cmd_input', 'words_input'])
        self.lis = ListenTool()

    def execute(self, userdata):
        rospy.loginfo('Executing state: STASK')
        if userdata.cmd_input == 'what_is_the_date_today':
            date = getDate()
            speak("It's " + date)
        elif userdata.cmd_input == 'what_time_is_it':
            time = getTime()
            speak("It's " + time)
        elif userdata.cmd_input == 'i_have_a_message':
            speak('Please speak a message')
            recordingMessage()
            speak('Repeat the message')
            playMessage('message.wav')
            if self.lis.isThisOK():
                speak('I save message')
            else:
                speak('fuck')
        elif userdata.cmd_input == 'start_happy_time':
            speak("Welcome to Tokyo Disneyland !!")
            bgmPlay('happy_time.mp3')
        elif userdata.cmd_input == 'finish_voice_control':
            playMessage('goodbye.wav')
            return 'finish_voice_control'
        return 'finish_stask'


class Learn(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_learn'],
                input_keys = ['cmd_input', 'words_input'])
        # Survice
        self.ggi_learning_srv = rospy.ServiceProxy('/ggi_learning', GgiLearning)
        self.location_setup_srv = rospy.ServiceProxy('/location_setup', LocationSetup)
        # Value
        self.file_name = 'none'
        self.lis = ListenTool()

    def execute(self, userdata):
        rospy.loginfo('Executing state: LEARN')
        speak(userdata.words_input)
        if userdata.cmd_input == 'add_location':
            location_name = 'shelf'# 場所名サーバー
            if self.lis.checkName(location_name):
                # self.location_setup_srv(state = 'add', name = location_name)
                speak('Location added')
            else:
                speak('Say the command again')
        elif userdata.cmd_input == 'save_location':
            self.file_name = 'gpsr'# 名前聞くサービス
            if self.lis.checkName(self.file_name):
                # self.location_setup_srv(state = 'save', name = self.file_name)
                speak('Location saved')
            else:
                speak('Say the command again')
        elif userdata.cmd_input == 'save_map':
            speak('Save the map as ' + self.file_name)
            if self.lis.canISave():
                # sp.Popen(['rosrun','map_server','map_saver','-f','/home/athome/map/'+ self.file_name])
                rospy.sleep(0.5)
                speak('Map saved')
            else:
                speak("Don't saved")
        elif userdata.cmd_input == 'start_ggi_learning':
            #self.ggi_learning_srv()
            speak('start ggi learning')
        return 'finish_learn'


class Event(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_event'],
                input_keys = ['cmd_input', 'words_input'])
        # Publisher
        self.pub_follow_req = rospy.Publisher('/chase/request', String, queue_size = 1)
        # ServiceProxy
        self.ap_listen_srv = rospy.ServiceProxy('/gpsr/actionplan', ActionPlan)
        self.ap_result = ActionPlan

    def execute(self, userdata):
        rospy.loginfo('Executing state: EVENT')
        speak(userdata.words_input)
        if userdata.cmd_input == 'start_follow':
            self.pub_follow_req.publish('start')
        elif userdata.cmd_input == 'stop_follow':
            self.pub_follow_req.publish('stop')
        elif userdata.cmd_input == 'start_map_creating':
            rospy.sleep(0.1)
            # sp.Popen(['roslaunch','turtlebot_navigation','gmapping_demo.launch'])
            rospy.sleep(0.5)
            # sp.Popen(['roslaunch','turtlebot_teleop','keyboard_teleop.launch'])
            rospy.sleep(0.5)
            # sp.Popen(['roslaunch','turtlebot_rviz_launchers','view_navigation.launch'])
        elif userdata.cmd_input == 'hey_mimi':
            self.ap_result = self.ap_listen_srv()
            if self.ap_result.result:
                action = self.ap_result.action
                data = self.ap_result.data
                print action
                print data
                exeActionPlanAC(action, data)
            else:
                speak("Say the command again")
        return 'finish_event'


def main():
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])

    with sm_top:
        smach.StateMachine.add(
                'LISTEN',
                Listen(),
                transitions = {'listen_failed':'LISTEN',
                               'motion':'MOTION',
                               'stask':'STASK',
                               'learn':'LEARN',
                               'event':'EVENT',
                               'stask':'STASK'},
                remapping = {'cmd_output':'cmd_name',
                             'words_output':'words'})

        smach.StateMachine.add(
                'MOTION',
                Motion(),
                transitions = {'finish_motion':'LISTEN'},
                remapping = {'cmd_input':'cmd_name',
                             'words_input':'words'})

        smach.StateMachine.add(
                'STASK',
                STask(),
                transitions = {'finish_stask':'LISTEN',
                               'finish_voice_control':'finish_sm_top'},
                remapping = {'cmd_input':'cmd_name',
                             'words_input':'words'})


        smach.StateMachine.add(
                'LEARN',
                Learn(),
                transitions = {'finish_learn':'LISTEN'},
                remapping = {'cmd_input':'cmd_name',
                             'words_input':'words'})

        smach.StateMachine.add(
                'EVENT',
                Event(),
                transitions = {'finish_event':'LISTEN'},
                remapping = {'cmd_input':'cmd_name',
                             'words_input':'words'})

    outcome = sm_top.execute()


if __name__ == '__main__':
    rospy.init_node('sm_voice_control', anonymous = True)
    main()
