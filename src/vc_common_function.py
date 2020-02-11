#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-----------------------------------------------------------
# Title: 音声コマンドから実行される機能を纏めたスクリプト
# Author: Issei Iida
# Date: 2020/02/12
# Memo: 
#-----------------------------------------------------------

# Python
import datetime
import pyaudio
import sys
import time
import wave

# ROS
import rospy
from ggi.srv import YesNo

chunk = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
RECORD_SECONDS = 3
OUTFILE = '/home/issei/catkin_ws/src/mimi_voice_control/wav_file/message.wav'


class ListenTool():
    def __init__(self):
        self.yesno_srv = rospy.ServiceProxy('/yes_no', YesNo)

    def isThisOK(self):
        speak('Is this ok ?')
        result = self.yesno_srv()
        return result.result

    def checkName(self, target_name):
        speak('Name is ' + target_name)
        speak('Is this corect ?')
        result = self.yesno_srv()
        return result.result

    def canISave(self):
        speak('Can I save?')
        result = self.yesno_srv()
        return result.result


class GetDateTime():
    def __init__(self):
        self.dt_now = datetime.datetime.now()

    def getDate(self):
        date_now = self.dt_now.strftime('%A,%B%d,%Y')
        print date_now
        return date_now

    def getTime(self):
        time_now = self.dt_now.strftime('%H,%M')
        print time_now
        return time_now


def recordingMessage():
    p = pyaudio.PyAudio()
    stream = p.open(
        format = FORMAT,
        channels = CHANNELS,
        rate = RATE,
        input = True,
        frames_per_buffer = chunk
    )

    # recording message
    print 'start recoding'
    all = []
    for i in range(0, int(RATE / chunk * RECORD_SECONDS)):
        data = stream.read(chunk)
        all.append(data)
    stream.close()
    p.terminate()
    print 'finish recoding'
    
    # save message
    data = b''.join(all)
    out = wave.open(OUTFILE,'w')
    out.setnchannels(1) # message
    out.setsampwidth(2) # 16bits
    out.setframerate(RATE)
    out.writeframes(data)
    out.close()
    print 'finish save'


def playMessage():
    try:
        wf = wave.open(OUTFILE, "r")
    except FileNotFoundError:
        print("[Error 404] No such file")

    # ストリームを開く
    p = pyaudio.PyAudio()
    stream = p.open(
            format = p.get_format_from_width(wf.getsampwidth()),
            channels = wf.getnchannels(),
            rate = wf.getframerate(),
            output = True)

    # 音声を再生
    # chunk = 1024
    data = wf.readframes(chunk)
    while data != '':
        stream.write(data)
        data = wf.readframes(chunk)
    stream.close()
    p.terminate()
