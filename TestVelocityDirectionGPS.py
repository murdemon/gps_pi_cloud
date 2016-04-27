#!/usr/bin/python

import os
import serial
import serial.tools.list_ports
import time
import sys
import math
import RPi.GPIO as GPIO
import pynmea2

from twisted.internet.task import LoopingCall
from twisted.internet import reactor


def convertGPSToXYZ(geoPoint):
    earthRadius = 3963.34
    Z = earthRadius*math.sin(math.radians(geoPoint[0]))
    r = math.sqrt( math.pow(earthRadius, 2) - math.pow(Z, 2) )
    Y = r * math.sin(math.radians(geoPoint[1]))
    X = math.sqrt( math.pow(earthRadius, 2) - math.pow(Y, 2) - math.pow(Z, 2) )
    if math.fabs(geoPoint[1])>90.0:
        X = -1.0 * X
    return [X, Y, Z]




def distanceTravelled(geoPoint1, geoPoint2):
    earthOne = convertGPSToXYZ(geoPoint1)
    earthTwo = convertGPSToXYZ(geoPoint2)
    return math.sqrt( math.pow(earthOne[0] - earthTwo[0], 2) + math.pow(earthOne[1] - earthTwo[1], 2) + math.pow(earthOne[2] - earthTwo[2], 2) )




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
    if inputLine[0:6] != '$GPRMC':
        print('Bad sentence type ' + inputLine[0:6])
        return False
    if inputLine[len(inputLine)-5:len(inputLine)-4] != '*':
        print('Bad ending asterisk ' + inputLine[len(inputLine)-5:len(inputLine)-4])
        return False
    calculatedCheckSum = 0
    for i in range(1,len(inputLine)-5):
        calculatedCheckSum = calculatedCheckSum ^ ord(inputLine[i])  # ^ is XOR
    checkSumString = hex(calculatedCheckSum).upper()
    checkSumString = checkSumString[2:4]
    if len(checkSumString) == 1:
        checkSumString = '0' + checkSumString
    if checkSumString != inputLine[len(inputLine)-4:len(inputLine)-2]:
        print('Mismatched check sums are: ' + checkSumString + ' and ' + inputLine[len(inputLine)-4:len(inputLine)-2])
        return False
    theSplit = inputLine.split(',')
    if (theSplit[2] != 'A'):
        print('Bad Validity')
        return False
    if (theSplit[4] != 'N') and (theSplit[4] != 'S'):
        print('Bad N S')
        return False
    try:
        degreesLatitude = float(theSplit[3]) / 100.0
        if theSplit[3] == 'S':
            degreesLatitude = -1.0*degreesLatitude
        geoPoint[0] = degreesLatitude    
    except ValueError:
        print('Latitude value error')
        return False  
    if (theSplit[6] != 'E') and (theSplit[6] != 'W'):
        print('Bad E W')
        return False
    try:
        degreesLongitude = float(theSplit[5]) / 100.0
        if theSplit[6] == 'E':
            degreesLongitude = -1.0*degreesLongitude
        geoPoint[1] = degreesLongitude    
    except ValueError:
        print('Longitude value error')
        return False
    try:
        theTime = float(theSplit[1])
        theTime = convertHHMMSSToTotalSeconds(theTime)
        print('The UTC Time (total seconds) is: ' + str(theTime))
        geoPoint[2] = theTime
    except ValueError:
        print('Bad UTC Time')
        return False
    try:
        theVelocity = float(theSplit[7])
        print('The Velocity is: ' + str(theVelocity))
        geoPoint[3] = theVelocity
    except ValueError:
        print('Bad Velocity')
        return False
    try:
        theBearing = float(theSplit[8])
        print('The Bearing is: ' + str(theBearing))
        geoPoint[4] = theBearing
    except ValueError:
        print('Bad Bearing')
        return False
    try:
        theDate = theSplit[9]
        theDate = theDate[2:4] + "/" + theDate[0:2] + "/20" + theDate[4:6]  # format the date as: mm/dd/yyyy
        print('The Date is: ' + str(theDate))
        geoPoint[5] = theDate
    except ValueError:
        print('Bad Date')
        return False
    return True


#---------------------------------------------------------------#
# Globals vars
#---------------------------------------------------------------#
timeDelay = 0
theTotalDistance = 0.0
theTotalDistanceToday = 0.0
counter = 1
theGPSDevice = serial.Serial()
previousGeoPoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currentGeoPoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

previousTime = previousGeoPoint[2]
theVelocity = previousGeoPoint[3] * 1.150777
theBearing = previousGeoPoint[4]
theDate = previousGeoPoint[5]
previousDate = theDate
theTotalDistance = 0.0
theTotalDistanceToday = 0.0
currentTime = 0.0
previousTime = currentTime



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
    print('Sorry but the GPS device is not plugged in')
    sys.exit()
 theGPSDevice = serial.Serial(port=thePortNumber, baudrate=4800, bytesize=8, stopbits=1, parity='N', xonxoff=False, timeout=0)
 print(thePortNumber)



#---------------------------------------------------------------#
# Start GPS positioning procedure
#---------------------------------------------------------------#
# GoodStart = False
# while (not GoodStart):
#    inputSentence = theGPSDevice.readline()
#    print(inputSentence)
#    inputLine = inputSentence
#    previousGeoPoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]     # Latitude  Longitude  Time  Velocity  Bearing  Date
#    if verifyVDB(inputLine, previousGeoPoint):
#        previousTime = previousGeoPoint[2]
#        theVelocity = previousGeoPoint[3] * 1.150777
#        theBearing = previousGeoPoint[4]
#        theDate = previousGeoPoint[5]
#        previousDate = theDate
#        GoodStart = True



theTotalDistance = 0.0
theTotalDistanceToday = 0.0
counter = 1
headerLine = 'Count:' + "," + 'Date (dd/mm/yyyy):' + "," + 'Time (decimal seconds):' + "," + 'Latitude (decimal degrees):' + "," + 'Longitude (decimal degrees):' + "," + 'Velocity (miles per hour):' + "," + 'Current Bearing (degrees):' + "," + 'Total Distance (miles):' + "," + 'Total Distance Today (miles):' + chr(13) + chr(10)
outputFile = open('LoggedData.csv', 'a')  # open log file
outputFile.write(headerLine)
#outputLine = str(counter) + "," + str(theDate) + "," + str(previousTime) + "," + str(previousGeoPoint[0]) + "," + str(previousGeoPoint[1]) + "," + format(theVelocity, '.1f') + "," + format(theBearing, '.1f') + "," + format(theTotalDistance, '.1f') + "," + format(theTotalDistanceToday, '.1f') + chr(13) + chr(10)
#outputFile.write(outputLine)
outputFile.close()
#print('Counter = ' + str(counter))
#counter = counter + 1

reader = pynmea2.NMEAStreamReader()
data = ""

def LoopProc():
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
 
 data = theGPSDevice.read()

 try:
  for msg in reader.next(data):
    print(msg)
    inputLine = str(msg) + "  "
    currentGeoPoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]    # Latitude  Longitude  Time  Velocity  Bearing  Date
    if verifyVDB(inputLine, currentGeoPoint):
        currentTime = currentGeoPoint[2]
        theVelocity = previousGeoPoint[3] * 1.150777    # convert knots to miles per hour
        theBearing = previousGeoPoint[4]
        theDate = currentGeoPoint[5]
        if ( (currentTime - previousTime) > timeDelay  ) and ( (currentGeoPoint[0] != previousGeoPoint[0]) or (currentGeoPoint[1] != previousGeoPoint[1]) ):
            theDistanceChange = distanceTravelled(previousGeoPoint, currentGeoPoint)
            if (theVelocity == 0.0):
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
            outputLine = str(counter) + "," + str(theDate) + "," + str(currentTime) + "," + str(currentGeoPoint[0]) + "," + str(currentGeoPoint[1]) + "," + format(theVelocity, '.1f') + "," + format(theBearing, '.1f') + "," + format(theTotalDistance, '.1f') + "," + format(theTotalDistanceToday, '.1f') + chr(13) + chr(10)
            outputFile = open('LoggedData.csv', 'a')  # open log file in append mode
            outputFile.write(outputLine)                       # log the data
            outputFile.close()                                 # keep the file closed, except when writing to it!
            outputFile = open('gpsnow.csv', 'w')      # open log file and erase any existing file first
            outputFile.write(headerLine)                       # first write the header
            outputFile.write(outputLine)                       # now write the last logged data line
            outputFile.close()                                 # keep the file closed, except when writing to it!
            print("Counter = " + str(counter))                 # print the value of the counter            
            counter = counter + 1                              # increment the counter only when we have a valid sentence
	    return 1

 except pynmea2.nmea.ParseError:
  print ("Can't parse");


def Check_WiFi():
  hostname = "www.rcofox.com"
  response = os.system("ping -c 1 " + hostname)
  if response == 0:
    pingstatus = "Network Active"
    return True
  else:
    pingstatus = "Network Error"
    return False

def Power_Off():
  response = os.system("sudo shutdown -h now")

def Cloud_Loop():
 return True

Stop_Counter = 0 

def Timers():
 global Stop_Counter
 global theVelocity
 print("Timer loop")
 if theVelocity < 1 and not GPIO.input(2):
	Stop_Counter = Stop_Counter + 1
 else:
	Stop_Counter = 0
 return True

Init()

Loopc = LoopingCall(LoopProc)
Loopc.start(0.001)

Loop_cloud = LoopingCall(Cloud_Loop)
Loop_cloud.start(20)

Loop_timers = LoopingCall(Timers)
Loop_timers.start(1)

reactor.run()
