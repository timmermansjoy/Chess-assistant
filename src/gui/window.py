
import sys
from PyQt5 import QtCore, QtGui, QtWidgets, uic, QtSvg
from PyQt5.QtCore import Qt

display_width = 1200
display_height = 1000


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.label = QtWidgets.QLabel()
        canvas = QtGui.QPixmap(660, 660)
        canvas.fill(QtGui.QColor("#ffffff"))
        self.label.setPixmap(canvas)
        self.setCentralWidget(self.label)
        self.setStyleSheet("background-color: white;")
        self.setGeometry(0, 0, display_width, display_height)
        self.draw_board()
        self.draw_piece()

    def draw_board(self):
        # variables
        width = int(600 / 8)
        height = int(600 / 8)
        whitecolor = "#ecd8c2"  # White on a normal chess board
        redcolor = "#ad5b4b"  # Black on a normal chess board

        painter = QtGui.QPainter(self.label.pixmap())

        for i in range(8):
            for j in range(8):
                if (j + i) % 2 == 0:
                    brush = QtGui.QBrush()
                    brush.setColor(QtGui.QColor(whitecolor))
                    brush.setStyle(Qt.SolidPattern)
                    painter.setBrush(brush)
                else:
                    brush = QtGui.QBrush()
                    brush.setColor(QtGui.QColor(redcolor))
                    brush.setStyle(Qt.SolidPattern)
                    painter.setBrush(brush)

                painter.drawRects(
                    QtCore.QRect((width * i) + 50, (height * j) + 50, width, height),
                )
        painter.end()

    def draw_piece(self):
        pass


app = QtWidgets.QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec_()
