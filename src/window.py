import sys
from PyQt5 import QtCore, QtGui, QtWidgets, uic, QtSvg
from PyQt5.QtCore import Qt


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.label = QtWidgets.QLabel()
        canvas = QtGui.QPixmap(900, 900)
        self.label.setPixmap(canvas)
        self.setCentralWidget(self.label)
        self.draw_board()
        self.draw_piece()

    def draw_board(self):
        width = int(800 / 8)
        height = int(800 / 8)
        painter = QtGui.QPainter(self.label.pixmap())

        for i in range(8):
            for j in range(8):
                if (j + i) % 2 == 0:
                    brush = QtGui.QBrush()
                    brush.setColor(QtGui.QColor("#FFFFFF"))
                    brush.setStyle(Qt.SolidPattern)
                    painter.setBrush(brush)
                else:
                    brush = QtGui.QBrush()
                    brush.setColor(QtGui.QColor("#000000"))
                    brush.setStyle(Qt.SolidPattern)
                    painter.setBrush(brush)

                painter.drawRects(
                    QtCore.QRect((width * i) + 50, (height * j) + 50, width, height),
                )
        painter.end()

    def draw_piece(self):
        brush = QtGui.QBrush()
        brush.setColor(QtGui.QColor("#2F5A25"))
        brush.setStyle(Qt.SolidPattern)
        painter = QtGui.QPainter(self.label.pixmap())
        painter.setBrush(brush)
        painter.drawRects(
            QtCore.QRect(60, 60, 200, 200),
        )
        painter.end()


app = QtWidgets.QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec_()
