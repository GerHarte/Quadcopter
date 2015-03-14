#!/usr/bin/env python
import cgi
form=cgi.FieldStorage()

import json
import sys
import time

import serial
ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
time.sleep(1)
ser.write(str(form["value"].value))


print "Content-type: application/json"
print
print(json.JSONEncoder().encode({"status":"ok"}))


