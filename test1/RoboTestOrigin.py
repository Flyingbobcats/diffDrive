#!/usr/bin/python

from Raspi_PWM_Servo_Driver import PWM
import time
import posq
import numpy as np
import socket
import time

UDP_IP = "192.168.0.195"
#UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

pwm = PWM(0x6F)

servoMin = 150  # Min pulse length out of 4096
servoMax = 600  # Max pulse length out of 4096

def getPos():
  data, addr = sock.recvfrom(128) # buffer size is 1024 bytes
  # print "received message:", data
  output = data.split(",")
  x = float(output[0])
  y = -float(output[1])
  heading = float(output[2])

  if heading < 0.0:
        heading += np.pi*2


  print(x,y,heading)
  return [x, y, heading]

def setServoPulse(channel, pulse):
  pulseLength = 1000000                   # 1,000,000 us per second
  pulseLength /= 60                       # 60 Hz
  print "%d us per period" % pulseLength
  pulseLength /= 4096                     # 12 bits of resolution
  print "%d us per bit" % pulseLength
  pulse *= 1000
  pulse /= pulseLength
  pwm.setPWM(channel, 0, pulse)

pwm.setPWMFreq(60)                        # Set frequency to 60 Hz

t = 0
xnow = getPos()
xend = [0,0,0]
direction = 1
old_beta = 0
vmax = 1
base =  .1075

while True:
  xnow = getPos()
  output = []
  output = posq.step(t,xnow,xend,direction,old_beta,vmax,base)

  #print(output)

  vl = output[0]
  vr = output[1]

  vl = (((vl)*(servoMax-375)/(1))+450)
  vr = (((vr)*(servoMax-375)/(1))+450)

  print(vl,vr)

  # Change speed of continuous servo on channel O
  pwm.setPWM(0, 0, int(vl))
  pwm.setPWM(1, 0, int(vr))
  #time.sleep(.5)

pwm.setPWM(0, 0, 0)
pwm.setPWM(0, 0, 0)