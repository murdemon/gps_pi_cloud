#!/usr/bin/python
import time as _time
import csv
import grequests
import os
import serial
import serial.tools.list_ports
import time
import sys
import math
import RPi.GPIO as GPIO
import pynmea2
import ConfigParser
from datetime import datetime
from datetime import timedelta
                                
from twisted.internet.task import LoopingCall
from twisted.internet import reactor
from requests_twisted import TwistedRequestsSession
import logging
import json
import ast

from gps_helper import updating_cloud
from gps_helper import verifyVDB
from gps_helper import log_set
from gps_helper import config_set
from gps_helper import We_on_home
from gps_helper import distanceTravelled
from gps_helper import save_csv

config = ConfigParser.RawConfigParser()
config.read('/home/pi/GPS/gps.conf')
config_set(config)

if len(sys.argv) == 1:
	logging.basicConfig(filename='/home/pi/GPS/gps.log',level=logging.INFO,format='%(asctime)s %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p')
logging.basicConfig(format='%(asctime)s %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p')
log = logging.getLogger()
if len(sys.argv) > 1:
	log.setLevel(logging.INFO)

log_set(log)

#------------------------------------------------------------#
# Globals vars
#---------------------------------------------------------------#
timeDelay = 0
theTotalDistance = 0.0
theTotalDistanceToday = 0.0
counter = 1

theGPSDevice = serial.Serial()
previousGeoPoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currentGeoPoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

previousTime = 0.0
theVelocity = 0.0
theBearing = 0.0
theDate = 0.0
previousDate = 0.0
currentTime = 0.0
previousTime = 0.0


outputFile = open('/home/pi/GPS/LoggedData.csv', 'a')  # open log file
reader = pynmea2.NMEAStreamReader()
data = ""

delay = 0
home = False
was_home = True
mesur_dist = False

Stop_Counter = 0


def Init():
 global timeDelay
 global theTotalDistance
 global theTotalDistanceToday
 global counter
 global theGPSDevice
 global previousGeoPoint
 global currentGeoPoint
 global previousTime
 global theVelocity
 global theBearing
 global theDate
 global previousDate
 global theTotalDistance
 global theTotalDistanceToday
 global currentTime
 global previousTime
 global mesur_dist
#----------------------------------------------------------------#
# Init Serial port
#----------------------------------------------------------------#
 timeDelay = 10
 GPIO.setmode(GPIO.BCM)
 GPIO.setup(2, GPIO.IN)

 thePortNumber = ''  # hopefully this will become non-empty later
 listOfCOMPorts = list(serial.tools.list_ports.comports())
 for i in range(len(listOfCOMPorts)):
    if listOfCOMPorts[i][1][0:12] == 'USB-Serial C':
      thePortNumber = listOfCOMPorts[i][0]      # get the active/used port number
 if thePortNumber == '':
    log.info('Sorry but the GPS device is not plugged in')
    sys.exit()
 theGPSDevice = serial.Serial(port=thePortNumber, baudrate=4800, bytesize=8, stopbits=1, parity='N', xonxoff=False, timeout=0)
 log.info(thePortNumber)
#-----------------------------------------------#
#  Read retain from file
#-----------------------------------------------#
 try:
# if True:
  outputFile_dist = open('/home/pi/GPS/retain', 'r')  # open log file
  line = outputFile_dist.readline()
  theTotalDistance = float(line)
  line = outputFile_dist.readline()
  mesur_dist = ast.literal_eval(line)
  outputFile_dist.close()
  log.info("RETAIN : Get Distance from saved file " + str(theTotalDistance))
  log.info("RETAIN : Get mesure from saved file " + str(mesur_dist))

 except:
  log.info("Error not get Retain")


def GPS_Loop():
 global timeDelay
 global theTotalDistance
 global theTotalDistanceToday
 global counter
 global theGPSDevice
 global previousGeoPoint
 global currentGeoPoint
 global previousTime
 global theVelocity
 global theBearing
 global theDate
 global previousDate
 global theTotalDistance
 global theTotalDistanceToday
 global currentTime
 global previousTime
 global reader
 global data
 global outputFile
 global mesur_dist
 global config

 data = theGPSDevice.read()
 theDistanceChange  = 0.0
 try:
  for msg in reader.next(data):
    log.info(msg)
    outputFile.write(str(msg) + chr(13) + chr(10))                       # log the data

    inputLine = str(msg) + "  "
    currentGeoPoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]    # Latitude  Longitude  Time  Velocity  Bearing  Date
    if verifyVDB(inputLine, currentGeoPoint):
        currentTime = currentGeoPoint[2]
        theVelocity = previousGeoPoint[3] * 1.150777    # convert knots to miles per hour
        theBearing = previousGeoPoint[4]
        theDate = currentGeoPoint[5]
   
        if ( (currentTime - previousTime) > timeDelay  ) and ( (currentGeoPoint[0] != previousGeoPoint[0]) or (currentGeoPoint[1] != previousGeoPoint[1]) ):
            theDistanceChange = distanceTravelled(previousGeoPoint, currentGeoPoint)
            if (theVelocity < 1.0):
                theDistanceChange =0.0;
            if (theDate != previousDate):
                theTotalDistanceToday = 0.0
            else:
                theTotalDistanceToday = theTotalDistanceToday + theDistanceChange
            theTotalDistance = theTotalDistance + theDistanceChange
            previousGeoPoint[0] = currentGeoPoint[0]
            previousGeoPoint[1] = currentGeoPoint[1]
            previousGeoPoint = currentGeoPoint
            previousTime = currentTime
            previousDate = theDate
            log.info("Added distance " + str(theDistanceChange) + " " + str(currentGeoPoint[0]) + "/" + str(currentGeoPoint[1]) + " - " + str(previousGeoPoint[0]) + "/" + str(previousGeoPoint[1]))
	    log.info("Counter = " + str(counter))                 # log.info the value of the counter            
            counter = counter + 1                              # increment the counter only when we have a valid sentence
	    return 1
    else:
	currentGeoPoint = previousGeoPoint	
 except:
  log.info ("Can't parse");
# log.info("Added distance " + str(theDistanceChange) + " " + str(currentGeoPoint[0]) + "/" + str(currentGeoPoint[1]) + " - " + str(previousGeoPoint[0]) + "/" + str(previousGeoPoint[1]))

def Logic_Loop():
 global home
 global was_home
 global theTotalDistance
 global mesur_dist
 global new_data
 global currentGeoPoint
 global config

 log.info("Cloud loop "+str(currentGeoPoint[0])+" "+str(currentGeoPoint[1])+" "+str(mesur_dist))

 if currentGeoPoint[0] != 0 and currentGeoPoint[1] != 0:
 	home = We_on_home(currentGeoPoint,config)
        if not home and was_home and not mesur_dist:
	   log.info("we go out from BASE !!!!!!!!!!!!!!!!")
	   mesur_dist = True
	   theTotalDistance = 0.0
	was_home = home
	
	if mesur_dist and theTotalDistance > 1.0 and home:
	   log.info("Going distance is " + str(theTotalDistance))
	   save_csv(theTotalDistance,config)
	   mesur_dist = False

        if mesur_dist and theTotalDistance <= 1.0 and home:
           log.info("Weon home with small distance (clear) is " + str(theTotalDistance))
           theTotalDistance = 0.0
	   mesur_dist = False

	if mesur_dist:
	   log.info(" Distance is " + str(theTotalDistance))

 return True



def Timers():
 global Stop_Counter
 global theVelocity
 global theTotalDistance
 global currentGeoPoint


 outputFile_dist = open('/home/pi/GPS/retain', 'w')  # open log file
 outputFile_dist.write(str(theTotalDistance)+ chr(13) + chr(10))
 outputFile_dist.write(str(mesur_dist)+ chr(13) + chr(10))
 outputFile_dist.close()


 if currentGeoPoint[0] != 0 and currentGeoPoint[1] != 0:
   Stop_Counter = Stop_Counter + 1

 if len(sys.argv) > 1:
  if Stop_Counter > 5 and Stop_Counter < 20:
	  home_latitude = config.set('conf','home_latitude','15')
	  home_longitude = config.set('conf','home_longitude','15')
	  log.info("SIMULLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL OUTTTTTT")
  if Stop_Counter > 10:
	  theTotalDistance = 500
  if Stop_Counter > 20:
          home_latitude = config.set('conf','home_latitude','34.026816')
          home_longitude = config.set('conf','home_longitude','118.296722')
	  log.info("SIMULLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL BACK")
 return True

def fail(f):
    log.info("we got an exception: %s" % (f.getTraceback(),))


Init()

Loop1 = LoopingCall(GPS_Loop)
Loop1.start(0.001).addErrback(fail)

Loop2 = LoopingCall(Logic_Loop)
Loop2.start(1).addErrback(fail)

Loop3 = LoopingCall(updating_cloud)
Loop3.start(1).addErrback(fail)

Loop4 = LoopingCall(Timers)
Loop4.start(1).addErrback(fail)

reactor.run()
