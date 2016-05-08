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
import ast

def str_color(color, data):
    colors = {'pink': '\033[95m', 'blue': '\033[94m', 'green': '\033[92m', 'yellow': '\033[93m', 'red': '\033[91m',
      'ENDC': '\033[0m', 'bold': '\033[1m', 'underline': '\033[4m'}

    return colors[color] + str(data) + colors['ENDC']

session = TwistedRequestsSession()
log = logging.getLogger()
was_wifi = False
new_data = 0
data_was_updated = 0
sending_in_progress = 0

csvfile = open('/home/pi/GPS/setSensorData.csv', 'ab')
fieldnames = ["Operation","Flag","ObjectId","ObjectType","MobileRecordId","Functional Group Name","Organization Name","Organization Number","Value","Datetime","Sensor Name","SensorRecordId"]
writer = csv.DictWriter(csvfile, fieldnames=fieldnames, quoting=csv.QUOTE_ALL)

dt_now_PLC =  datetime.now()

config  = ConfigParser.RawConfigParser()

def config_set(config_main):
    global config
    config = config_main

def log_set(log_main):
    global log
    log = log_main

def convertGPSToXYZ(geoPoint):
    earthRadius = 3963.34
    Z = earthRadius*math.sin(math.radians(geoPoint[0]))
    X = earthRadius*math.cos(math.radians(geoPoint[0]))*math.cos(math.radians(geoPoint[1]))
    Y = earthRadius*math.cos(math.radians(geoPoint[0]))*math.sin(math.radians(geoPoint[1]))
    if math.fabs(geoPoint[1])>90.0:
        X = -1.0 * X
    return [X, Y, Z]

def distanceTravelled(geoPoint1, geoPoint2):
    earthOne = convertGPSToXYZ(geoPoint1)
    earthTwo = convertGPSToXYZ(geoPoint2)
    return math.sqrt( math.pow(earthOne[0] - earthTwo[0], 2) + math.pow(earthOne[1] - earthTwo[1], 2) + math.pow(earthOne[2] - earthTwo[2], 2) )



def ConvToDecim(cord):
    deg = int(float(cord)/100)
    min = int(float(cord)-float(deg)*100)
    sec = ((float(cord)-float(deg)*100-float(min))*60)
    decim = float(deg)+float(min)/60+float(sec)/3600
    return decim

def convertHHMMSSToTotalSeconds(inputTime):
    # the original input time is in the format hhmmss.xyz
    seconds = math.modf(inputTime / 100.0)
    # seconds is now a 2-tuple containing (.ssxyz, hhmm.)
    minutes = math.modf(seconds[1] / 100.0)
    # minutes is now a 2-tuple containing (.mm, hh)
    return (seconds[0] * 100.0) + ((minutes[0] * 100.0) * 60.0) + (minutes[1] * 3600.0)




def verifyVDB(inputLine, geoPoint):
#  inputLine = "$GPRMC,025100.000,A,3403.5535,N,11833.0495,W,1.16,275.11,010915,,,A*72 "
#  perform the beginning and the ending line checks before checking any of the interior data values
#  because all the interior data values are obtained after splitting the data
    global dt_now_PLC
    if inputLine[0:6] != '$GPRMC':
        log.debug('Bad sentence type ' + inputLine[0:6])
        return False
    if inputLine[len(inputLine)-5:len(inputLine)-4] != '*':
        log.debug('Bad ending asterisk ' + inputLine[len(inputLine)-5:len(inputLine)-4])
        return False
    calculatedCheckSum = 0
    for i in range(1,len(inputLine)-5):
        calculatedCheckSum = calculatedCheckSum ^ ord(inputLine[i])  # ^ is XOR
    checkSumString = hex(calculatedCheckSum).upper()
    checkSumString = checkSumString[2:4]
    if len(checkSumString) == 1:
        checkSumString = '0' + checkSumString
    if checkSumString != inputLine[len(inputLine)-4:len(inputLine)-2]:
        log.debug('Mismatched check sums are: ' + checkSumString + ' and ' + inputLine[len(inputLine)-4:len(inputLine)-2])
        return False
    theSplit = inputLine.split(',')
    if (theSplit[2] != 'A'):
        log.debug('Bad Validity')
        return False
    if (theSplit[4] != 'N') and (theSplit[4] != 'S'):
        log.debug('Bad N S')
        return False
    try:
        degreesLatitude = float(theSplit[3]) / 100.0
	degreesLatitude = ConvToDecim(theSplit[3])
        if theSplit[3] == 'S':
            degreesLatitude = -1.0*degreesLatitude
        geoPoint[0] = degreesLatitude    
    except ValueError:
        log.debug('Latitude value error')
        return False  
    if (theSplit[6] != 'E') and (theSplit[6] != 'W'):
        log.debug('Bad E W')
        return False
    try:
        degreesLongitude = float(theSplit[5]) / 100.0
        degreesLongitude = ConvToDecim(theSplit[5])
	if theSplit[6] == 'W':
            degreesLongitude = -1.0*degreesLongitude
        geoPoint[1] = degreesLongitude    
    except ValueError:
        log.debug('Longitude value error')
        return False
    try:
        theTime = float(theSplit[1])
        log.debug('The UTC Time is: ' + str(theTime))
	theTime = convertHHMMSSToTotalSeconds(theTime)
        log.debug('The UTC Time (total seconds) is: ' + str(theTime))
        geoPoint[2] = theTime
    except ValueError:
        log.debug('Bad UTC Time')
        return False
    try:
        theVelocity = float(theSplit[7])
        log.debug('The Velocity is: ' + str(theVelocity))
        geoPoint[3] = theVelocity
    except ValueError:
        log.debug('Bad Velocity')
        return False
    try:
        theBearing = float(theSplit[8])
        log.debug('The Bearing is: ' + str(theBearing))
        geoPoint[4] = theBearing
    except ValueError:
        log.debug('Bad Bearing')
        return False
    try:
        theDate = theSplit[9]
        theDate = theDate[2:4] + "/" + theDate[0:2] + "/20" + theDate[4:6]  # format the date as: mm/dd/yyyy
        log.debug('The Date is: ' + str(theDate))
        geoPoint[5] = theDate
    except ValueError:
        log.debug('Bad Date')
        return False

    theDate = theSplit[9]
    theTime = theSplit[1]
    Month = theDate[2:4]
    Day = theDate[0:2]
    Year = "20" + theDate[4:6]
    Hour = theTime[0:2]
    Min = theTime[2:4]
    Sec = theTime[4:6]
    new_datetime = (int(Year), int(Month), int(Day), int(Hour), int(Min), int(Sec),int(0))
    dt_now_PLC = datetime( *new_datetime[:6])
    dt =dt_now_PLC - timedelta(hours=7)
    dt_now_PLC = dt
    log.debug("Date Time UTC " + str(dt_now_PLC))
    return True    

def Check_WiFi():
  hostname = "www.rcofox.com"
  response = os.system("ping -c 1 " + hostname + " 1>/dev/null")
  if response == 0:
    pingstatus = "Network Active"
    return True
  else:
    pingstatus = "Network Error"
    return False

def Power_Off():
  response = os.system("sudo shutdown -h now")


def save_csv(val,config):
	global dt_now_PLC
	global new_data
	global sending_in_progress
	
	if sending_in_progress == 0:
		now = dt_now_PLC
		dt = dt_now_PLC
		timestamp = _time.mktime(dt.timetuple())
		datetime_now = datetime.strftime(dt, "%Y-%m-%d %H:%M:%S")
		log.info("write to CSV for  Distance: "+str(val))                                    
		writer.writerow({"Operation": config.get('conf','Operation'),\
				 "Flag": config.get('conf','Flag'),\
				 "ObjectId": config.get('conf','ObjectId'),\
                                 "ObjectType": config.get('conf','ObjectType'),\
                                 "MobileRecordId": "SensorData"+"-"+str(timestamp),\
                                 "Functional Group Name": config.get('conf','Functional Group Name'),\
                                 "Organization Name": config.get('conf','Organization Name'),\
                                 "Organization Number": config.get('conf','Organization Number'),\
                                 "Value": 0 if val == -9999 else str('%.2f' % val),\
                                 "Datetime": datetime_now,\
                                 "Sensor Name": config.get('conf','Sensor Name'),\
                                 "SensorRecordId": config.get('conf','SensorRecordId')\
				})
		new_data = 1

def updating_cloud():
    global csvfile
    global writer
    global new_data
    global session
    global data_was_updated
    global sending_in_progress
    global delay
    global config
    global sending_in_progress
    global was_wifi

    log.info(str_color("pink",  "[POST loop] New data to cloud = " + str(bool(new_data))))

    file_emty = os.stat('/home/pi/GPS/setSensorData.csv').st_size==0
    is_wifi = Check_WiFi()
    if not file_emty and is_wifi and not was_wifi and  sending_in_progress == 0:
	log.info("We on Wifi and file not emty");
	new_data = 1
    was_wifi = is_wifi

    post_url = config.get('conf','post_url')
    if new_data == 1 and sending_in_progress == 0:
        csvfile.close()
	sending_in_progress = 1
	#-----------------------------------------------#
	# if have ne data make API setSensorData
	#-----------------------------------------------#
        log.info('Upload data to Cloud')
        csvfile = open('/home/pi/GPS/setSensorData.csv', 'rb')
	multiple_files = [('text', ('setSensorData.csv', csvfile , 'text/plain'))]
        r = session.post(post_url, files=multiple_files, timeout=20, stream=True)
	r.addCallback(print_status)
	r.addErrback(handleFailure) 

    if data_was_updated == 1:   
	#----------------------------------------------------------------#
	# Ask command for devices (first getState then if ok resetState)
	#----------------------------------------------------------------#
           log.info("Data was updatedin CSV or Cloud")
	   data_was_updated = 0

def We_on_home(currentGeoPoint,config):
  home_latitude = config.get('conf','home_latitude')
  home_longitude = config.get('conf','home_longitude')
  lat_diff = abs(float(home_latitude) - currentGeoPoint[0])
  log_diff = abs(float(home_longitude) -  currentGeoPoint[1])
  log.debug("Home check "+str(currentGeoPoint[0])+"/"+str(currentGeoPoint[1])+" "+str(home_latitude)+"/"+str(home_longitude)+" "+str(lat_diff)+"/"+str(log_diff))
  if lat_diff < 0.0005 and log_diff < 0.0005:
    log.info(str_color("green", str(dt_now_PLC)+' WIFI We at home!!!!!'))
    return True
  else:
    log.info(str_color("pink",  str(dt_now_PLC)+' WIFI We not home!!!!!!'))
    return False

def handleFailure(f):
         global csvfile
         global writer
         global new_data
         global data_was_updated
         global sending_in_progress

	 csvfile.close()
	 sending_in_progress = 0 
	 log.info("Timeout POST Sensor data to Cloud ")
         csvfile = open('/home/pi/GPS/setSensorData.csv', 'ab')
         data_was_updated = 1 
	 new_data = 0
         writer = csv.DictWriter(csvfile, fieldnames=fieldnames, quoting=csv.QUOTE_ALL)

def print_status(r):
 		global csvfile
    		global writer
    		global new_data
		global data_was_updated
		global sending_in_progress

		csvfile.close()
		sending_in_progress = 0
                log.info('Status: '+str(r.status_code))
		log.info('Body: '+str(r.text))

                if r.status_code == 200:
                        csvfile = open('/home/pi/GPS/setSensorData.csv', 'wb')
                        new_data = 0
                        data_was_updated = 1
	                writer = csv.DictWriter(csvfile, fieldnames=fieldnames, quoting=csv.QUOTE_ALL)
                elif r.status_code == 404:
                        csvfile = open('/home/pi/GPS/setSensorData.csv', 'ab')
                        new_data = 0
                	data_was_updated = 1
		else:
                        csvfile = open('/home/pi/setSensorData.csv', 'ab')
                        new_data = 0
			data_was_updated = 1
	        writer = csv.DictWriter(csvfile, fieldnames=fieldnames, quoting=csv.QUOTE_ALL)
