import re
import sys
import serial
import argparse
from select import select
import numpy as np
from time import sleep
from collections import deque, OrderedDict

import matplotlib.pyplot as plt
import matplotlib.animation as animation

    
class SerialVariablesParser():
    VAR_FLAG = '_DD_'
    RE_TS = re.compile(r'\((\d+)\)')
    RE_VARS = re.compile(r'(\w+): ([-\d.]+)')
    maxLen = 0

    def __init__(self, strPort, maxLen):
        # open serial port
        self.maxLen = maxLen
        self.ser = serial.Serial(strPort, 115200)
        self.vars = {}
        self.ts = deque([0.0] * self.maxLen)

    def update(self):
        line = self.ser.readline().decode('utf-8')        
                
        if self.VAR_FLAG in line:
            ts = self.RE_TS.search(line)
            self.addToBuf(self.ts, ts)
            
            for (varname, value) in self.RE_VARS.findall(line):
                value = float(value)
                if self.vars.get(varname) is None:
                    self.vars[varname] = deque([0.0] * self.maxLen)
                self.addToBuf(self.vars[varname], value)
        #else:
        print(line)
  
    def addToBuf(self, buf, val):
      if len(buf) < self.maxLen:
          buf.append(val)
      else:
          buf.pop()
          buf.appendleft(val)

    def send(self, cmd):
        self.ser.write((cmd).encode('utf-8'))
        print((cmd).encode('utf-8'))
        
    def close(self):
        # close serial
        self.ser.flush()
        self.ser.close()


class Plotter:
    def __init__(self, axes, port, maxlen):
        self.parser = SerialVariablesParser(port, maxlen)
        self.axes = axes
        self.plots = OrderedDict({})
        self.maxLen = maxlen
        #self.parser.update()
        #print(parser.vvars)
        
    def read_cmd(self):
        dr,dw,de = select([sys.stdin], [], [], 0)
        if dr != []:
            self.parser.send(sys.stdin.readline())
                
    def update(self, frame):
        self.read_cmd()
        self.parser.update()
        for var in self.parser.vars:
            if var in ['target', 'angle']:
                if self.plots.get(var) is None:
                    self.plots[var], = self.axes.plot([], [])
                self.plots[var].set_data(range(self.maxLen), self.parser.vars[var])
            
        return self.plots.values()
    
    def close(self):
        self.parser.close()
        
    
def main():
  # create parser
  parser = argparse.ArgumentParser(description="LDR serial")
  # add expected arguments
  parser.add_argument('--port', dest='port', default='/dev/tty.usbmodem2101', required=False)
#   parser.add_argument('--vars', nargs='*', dest='vars', required=True)

  # parse args
  args = parser.parse_args()
  
  #strPort = '/dev/tty.usbserial-A7006Yqh'
  strPort = args.port

  print('reading from serial port %s...' % strPort)



  print('plotting data...')

  MAX_LEN = 500
  # set up animation
  fig = plt.figure()
  axes = plt.axes(xlim=(0, MAX_LEN), ylim=(0, 360))
  plotter = Plotter(axes, strPort, MAX_LEN)
  #a0, = ax.plot([], [])  
  #a1, = ax.plot([], [])
  anim = animation.FuncAnimation(fig, plotter.update, 
                                #fargs=(a0, a1), 
                                 interval=50)

  # show plot
  plt.show()
  
  # clean up
  plotter.close()

  print('exiting.')
  

# call main
if __name__ == '__main__':
  main()
