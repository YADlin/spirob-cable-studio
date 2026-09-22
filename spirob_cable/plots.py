"""A large dual-cable plot with an optional tracking-error panel."""
from collections import deque
import math
from PySide6 import QtCore, QtWidgets
import pyqtgraph as pg

COLORS = ['#00867f','#d17720']


class Plots(QtWidgets.QWidget):
    def __init__(self):
        super().__init__(); self.records = [deque(maxlen=8000),deque(maxlen=8000)]
        self.frozen = False
        layout = QtWidgets.QVBoxLayout(self); layout.setContentsMargins(0,0,0,0)
        self.graph = pg.PlotWidget(background='white'); self.graph.setMinimumHeight(260)
        self.graph.addLegend(offset=(12,12)); self.graph.setLabel('bottom','Session time',units='s')
        self.graph.showGrid(x=True,y=True,alpha=.15)
        self.error = pg.PlotWidget(background='white'); self.error.setMinimumHeight(100)
        self.error.setMaximumHeight(160); self.error.setLabel('left','Target − encoder',units='mm')
        self.error.setLabel('bottom','Session time',units='s'); self.error.setXLink(self.graph)
        self.error.showGrid(x=True,y=True,alpha=.15)
        self.curves, self.targets, self.errors = [],[],[]
        for i,color in enumerate(COLORS):
            self.curves.append(self.graph.plot(pen=pg.mkPen(color,width=2),name=f'Cable {i+1} actual'))
            self.targets.append(self.graph.plot(pen=pg.mkPen(color,width=1.5,style=QtCore.Qt.PenStyle.DashLine),name=f'Cable {i+1} target'))
            self.errors.append(self.error.plot(pen=pg.mkPen(color,width=1.5)))
        for plot in (self.graph,self.error):
            plot.getAxis('left').enableAutoSIPrefix(False); plot.getAxis('bottom').enableAutoSIPrefix(False)
        layout.addWidget(self.graph,1); layout.addWidget(self.error)
        self.error.hide()

    def clear(self):
        for data in self.records: data.clear()
        for curve in self.curves+self.targets+self.errors: curve.setData([],[])

    def append(self, row):
        self.records[row['axis']].append(row)

    def draw(self, frame='connection'):
        if self.frozen: return
        fields = {'connection':('connection_mm','target_connection_mm','Displacement from connection'),
                  'rest':('offset_mm','target_mm','Displacement from physical rest'),
                  'absolute':('absolute_mm','target_absolute_mm','Estimated cable length')}
        actual,target,title = fields[frame]
        self.graph.setLabel('left',title,units='mm')
        for i,rows in enumerate(self.records):
            xs,ys,targets,errors = [],[],[],[]
            prev_epoch = None
            for row in rows:
                xs.append(row['read_end_s'])
                gap = frame != 'connection' and prev_epoch is not None and prev_epoch != row.get('reference_epoch')
                ys.append(math.nan if gap or row.get(actual) is None else row[actual])
                targets.append(math.nan if gap or row.get(target) is None else row[target])
                errors.append(math.nan if row.get('error_mm') is None else row['error_mm'])
                prev_epoch = row.get('reference_epoch')
            self.curves[i].setData(xs,ys,connect='finite',symbol='o' if len(xs)==1 else None,symbolSize=4)
            self.targets[i].setData(xs,targets,connect='finite')
            self.errors[i].setData(xs,errors,connect='finite')

    def auto_range(self):
        self.graph.enableAutoRange(); self.error.enableAutoRange()
