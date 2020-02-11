#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pyaudio
import sys
import time
import wave

chunk = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
RECORD_SECONDS = 30

p = pyaudio.PyAudio()

stream = p.open(
    format = FORMAT,
    channels = CHANNELS,
    rate = RATE,
    input = True,
    frames_per_buffer = chunk
)

all = []
print 'start recoding'
for i in range(0, int(RATE / chunk * RECORD_SECONDS)):
    data = stream.read(chunk)
    all.append(data)

stream.close()
p.terminate()
print 'finish recoding'

data = b''.join(all)
out = wave.open('mono.wav','w')
out.setnchannels(1) #mono
out.setsampwidth(2) #16bits
out.setframerate(RATE)
out.writeframes(data)
out.close()
print 'finish save'


try:
    wf = wave.open('mono.wav', "r")
except FileNotFoundError: #ファイルが存在しなかった場合
    print("[Error 404] No such file or directory: " + Filename)
    
# ストリームを開く
p = pyaudio.PyAudio()
stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                channels=wf.getnchannels(),
                rate=wf.getframerate(),
                output=True)

print 'start saisei'
# 音声を再生
chunk = 1024
data = wf.readframes(chunk)
while data != '':
    stream.write(data)
    data = wf.readframes(chunk)
stream.close()
p.terminate()
print ('all finish')
