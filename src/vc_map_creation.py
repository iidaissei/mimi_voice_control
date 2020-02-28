#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
# Title: マップ作成専用の音声コントロールプログラム
# Author: Issei Iida
# Date: 2020/02/26
# Memo: 機体の操作はteleopから行う仕様
#---------------------------------------------------------------------

# Python
import sys
import subprocess as sp
# ROS
import rospy
import smach
import smach_ros
from voice_common_pkg.srv import ListenCommand, YesNo
from mimi_common_pkg.srv import LocationSetup

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_function import speak

class Listen(smach.State): 
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['listen_failure',
                                         'listen_success'],
                             output_keys = ['cmd_output'])
        # Service
        self.listen_cmd_srv = rospy.ServiceProxy('/listen_command', ListenCommand)

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN')
        result = self.listen_cmd_srv(file_name = 'voice_cmd')
        if result.result == True:
            command = result.cmd
            userdata.cmd_output = command
            return 'listen_success'
        else:
            speak("I could't hear")
            return 'listen_failure'

class ExeCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['exe_cmd_finish', 'vc_finish'],
                             input_keys = ['cmd_input'])
        # Service
        self.listen_cmd_srv = rospy.ServiceProxy('/listen_command', ListenCommand)
        self.location_setup_srv = rospy.ServiceProxy('/location_setup', LocationSetup)
        self.yesno_srv = rospy.ServiceProxy('/yes_no', YesNo)

    def checkName(self, name):
        speak('Name is ' + name)
        speak('Is this OK')
        result = self.yesno_srv().result
        return result

    def execute(self, userdata):
        rospy.loginfo('Executing state: EXE_COMMAND')
        if userdata.cmd_input == 'append location name':
            location_name = self.listen_cmd_srv (file_name = 'location_name').cmd
            if self.checkName(location_name):
                self.location_setup_srv(state = 'add', name = location_name)
                speak('Added location name')
            else:
                speak('Say the command again')
        elif userdata.cmd_input == 'save location':
            file_name = self.listen_cmd_srv(file_name = 'map_name').cmd
            if self.checkName(file_name):
                self.location_setup_srv(state = 'save', name = file_name)
                speak('Saved location')
                sp.Popen(['rosrun','map_server','map_saver','-f','/home/athome/map/'+ file_name])
                rospy.sleep(0.5)
                speak('Saved map as ' + file_name)
            else:
                speak('Added location name')
        elif userdata.cmd_input == 'finish voice control':
            speak('Bye')
            return 'vc_finish'
        else:
            pass

def main():
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    with sm_top:
        smach.StateMachine.add(
                'LISTEN',
                Listen(),
                transitions = {'listen_success':'EXE_COMMAND',
                               'listen_failed':'LISTEN'},
                remapping = {'cmd_output':'cmd_name'})

        smach.StateMachine.add(
                'EXE_COMMAND',
                ExeCommand(),
                transitions = {'exe_cmd_finish':'LISTEN',
                               'vc_finish':'finish_sm_top'},
                remapping = {'cmd_input':'cmd_name'})

    outcome = sm_top.execute()

if __name__ == '__main__':
    rospy.init_node('mc_voice_control', anonymous = True)
    main()
