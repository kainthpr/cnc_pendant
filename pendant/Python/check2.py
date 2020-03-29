#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Test the speed of rapidly updating multiple plot curves
"""


from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
from pyqtgraph.ptime import time
import serial 
#QtGui.QApplication.setGraphicsSystem('raster')
app = QtGui.QApplication([])
#mw = QtGui.QMainWindow()
#mw.resize(800,800)
sp = serial.Serial(port = "COM6", baudrate=115200, bytesize=8, timeout=0, stopbits=serial.STOPBITS_ONE)


p = pg.plot()
p.setWindowTitle('pyqtgraph example: MultiPlotSpeedTest')
#p.setRange(QtCore.QRectF(0, -10, 5000, 20)) 
p.setLabel('bottom', 'Index', units='B')

nPlots = 1
nSamples = 500
#curves = [p.plot(pen=(i,nPlots*1.3)) for i in range(nPlots)]

c = pg.PlotCurveItem()
p.addItem(c)
p.setYRange(0, 6)
p.setXRange(0, nSamples)
p.resize(600,900)

rgn = pg.LinearRegionItem([nSamples/5.,nSamples/3.])
p.addItem(rgn)


lastTime = time()
fps = None
count = 0
data = [0]*300
def update():
    global c, data, p, lastTime, fps, nPlots, count
    count += 1
    #print "---------", count
    data[np.random.randint(0, 300)] = np.random.random()
    c.setData(data)
        
    now = time()
    dt = now - lastTime
    lastTime = now
    if fps is None:
        fps = 1.0/dt
    else:
        s = np.clip(dt*3., 0, 1)
        fps = fps * (1-s) + (1.0/dt) * s
    p.setTitle('%0.2f fps' % fps)
    #app.processEvents()  ## force complete redraw for every plot

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(0)
    
while 1:
    pass

## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()