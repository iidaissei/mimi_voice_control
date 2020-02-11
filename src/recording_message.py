#!/usr/bin/env python
# -*- coding: utf-8 -*-
#----------------------------------------------------------
# Title: 伝言を録音して再生もするファイル（お試し）
# Author: Issei Iida
# Date: 2020/02/12
#----------------------------------------------------------

# Python
import pyaudio
import sys
import time
import wave


chunk = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
RECORD_SECONDS = 3
OUTFILE = '/home/issei/catkin_ws/src/mimi_voice_control/wav_file/message.wav'


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


if __name__ == '__main__':
    recordingMessage()
    time.sleep(2.0)
    playMessage()
