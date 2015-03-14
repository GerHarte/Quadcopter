import sys, serial
import numpy as np
from time import sleep
from collections import deque
from matplotlib import pyplot as plt

class AnalogData:
  def __init__(self, maxLen):
    self.ax = deque([0.0]*maxLen)
    self.ay = deque([0.0]*maxLen)
    self.maxLen = maxLen

  def addToBuf(self, buf, val):
    if len(buf) < self.maxLen:
      buf.append(val)
    else:
      buf.pop()
      buf.appendleft(val)

  def add(self, data):
    assert(len(data) == 2)
    self.addToBuf(self.ax, data[0])
    self.addToBuf(self.ay, data[1])

class AnalogPlot:
  def __init__(self, analogData):
    plt.ion()
    self.axline, = plt.plot(analogData.ax)
    self.ayline, = plt.plot(analogData.ay)
    plt.ylim([0,1023])

  def update(self, analogData):
    self.axline.set_ydata(analogData.ax)
    self.ayline.set_ydata(analogData.ay)
    plt.draw()

def main():
  if(len(sys.argv) != 2):
    print 'Example usage: python showdata.py "/dev/ttyACM1"'
    exit(1)

  strPort = sys.argv[1];

  analogData = AnalogData(100)
  analogPlot = AnalogPlot(analogData)

  print 'plotting data...'
  
  ser = serial.Serial(strPort, 9600)
  while True:
    try:
      line = ser.readline()
      data = [float(val) for val in line.split()]
      if(len(data)==2):
        analogData.add(data)
        analogPlot.update(analogData)
    except KeyboardInterrupt:
      print 'exiting'
      break

  ser.flush()
  ser.close()

if __name__ == '__main__':
  main()