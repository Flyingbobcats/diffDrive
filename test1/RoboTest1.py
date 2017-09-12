#!/usr/bin/python

from Raspi_PWM_Servo_Driver import PWM
import time
import posq
import numpy as np

with open("traj.txt",'r') as trajfile:
	trajarr = np.genfromtxt(trajfile, dtype=None)


# ===========================================================================
# Example Code
# ===========================================================================

# Initialise the PWM device using the default address
# bmp = PWM(0x40, debug=True)
pwm = PWM(0x6F)

servoMin = 150  # Min pulse length out of 4096
servoMax = 600  # Max pulse length out of 4096

def setServoPulse(channel, pulse):
  pulseLength = 1000000                   # 1,000,000 us per second
  pulseLength /= 60                       # 60 Hz
  print "%d us per period" % pulseLength
  pulseLength /= 4096                     # 12 bits of resolution
  print "%d us per bit" % pulseLength
  pulse *= 1000
  pulse /= pulseLength
  pwm.setPWM(channel, 0, pulse)
  pwm.setPWM(channel, 1, pulse)

pwm.setPWMFreq(60)                        # Set frequency to 60 Hz

t = 0
xnow = [0,0,0]
xend = trajarr
direction = 1
old_beta = 0
vmax = 1
base = .4


for pos in range(0,len(xend)):

  output = []
  output = posq.step(t,xnow,xend[pos],direction,old_beta,vmax,base)
  xnow = xend[pos]

  #print(output)

  vl = output[0]
  vr = output[1]

  vl = (((vl)*(servoMax-375)/(1))+450)
  vr = (((vr)*(servoMax-375)/(1))+450)

  print(vl,vr)

  # Change speed of continuous servo on channel O
  pwm.setPWM(0, 0, int(vl))
  pwm.setPWM(1, 0, int(vr))
  time.sleep(.5)

pwm.setPWM(0, 0, 430)
pwm.setPWM(0, 0, 430)
