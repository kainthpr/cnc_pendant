import serial
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np
from random import randrange, uniform

sp = serial.Serial(port = "COM6", baudrate=115200, bytesize=8, timeout=0, stopbits=serial.STOPBITS_ONE)
timer = pg.QtCore.QTimer()
pw = pg.plot()
x = []
y = []
c = 0
max_len = 500

def update():
    global c, x, y
    while True:
        line = sp.readline()
        if line != '':
            y.append(int(line.split(" ")[2]))
            x.append(c)
            c+=1
            if len(x) > max_len:
                x= x[len(x)-max_len:]
                y= y[len(y)-max_len:]
        else:
            break
    pw.plot(x, y, clear=True)
    
timer.timeout.connect(update)

### MAIN PROGRAM #####
timer.start(0)


if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

