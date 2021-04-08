import sys
from PyQt5 import QtCore, QtGui, QtWidgets, uic, QtSvg
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QLineEdit, QGridLayout
from PyQt5.QtGui import QPixmap


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.title = 'ChessGUI'

        self.whiteBishopImg = QPixmap('src/resources/WhiteBishop.png')
        self.whiteBishopImg = self.whiteBishopImg.scaled(50, 50, Qt.KeepAspectRatio)
        self.blackBishopImg = QPixmap('src/resources/BlackBishop.png')
        self.blackBishopImg = self.blackBishopImg.scaled(50, 50, Qt.KeepAspectRatio)
        self.whiteRookImg = QPixmap('src/resources/WhiteRook.png')
        self.whiteRookImg = self.whiteRookImg.scaled(50, 50, Qt.KeepAspectRatio)
        self.blackRookImg = QPixmap('src/resources/BlackRook.png')
        self.blackRookImg = self.blackRookImg.scaled(50, 50, Qt.KeepAspectRatio)
        self.whiteKnightImg = QPixmap('src/resources/WhiteKnight.png')
        self.whiteKnightImg = self.whiteKnightImg.scaled(50, 50, Qt.KeepAspectRatio)
        self.blackKnightImg = QPixmap('src/resources/BlackKnight.png')
        self.blackKnightImg = self.blackKnightImg.scaled(50, 50, Qt.KeepAspectRatio)
        self.whitePawnImg = QPixmap('src/resources/WhitePawn.png')
        self.whitePawnImg = self.whitePawnImg.scaled(50, 50, Qt.KeepAspectRatio)
        self.blackPawnImg = QPixmap('src/resources/BlackPawn.png')
        self.blackPawnImg = self.blackPawnImg.scaled(50, 50, Qt.KeepAspectRatio)
        self.whiteKingImg = QPixmap('src/resources/WhiteKing.png')
        self.whiteKingImg = self.whiteKingImg.scaled(50, 50, Qt.KeepAspectRatio)
        self.blackKingImg = QPixmap('src/resources/BlackKing.png')
        self.blackKingImg = self.blackKingImg.scaled(50, 50, Qt.KeepAspectRatio)
        self.whiteQueenImg = QPixmap('src/resources/WhiteQueen.png')
        self.whiteQueenImg = self.whiteQueenImg.scaled(50, 50, Qt.KeepAspectRatio)
        self.blackQueenImg = QPixmap('src/resources/BlackQueen.png')
        self.blackQueenImg = self.blackQueenImg.scaled(50, 50, Qt.KeepAspectRatio)

        self.height = 1000
        self.width = 1200
        self.left = 15
        self.top = 15
        self.initUI()
        self.draw_piece()

    def paintEvent(self, event):
        width = int(600 / 8)
        height = int(600 / 8)
        whitecolor = "#ecd8c2"  # White on a normal chess board
        redcolor = "#ad5b4b"  # Black on a normal chess board
        greycolor = "#A4A2B8"
        nocolor = "#FCFFE9"

        painter = QtGui.QPainter(self)

        for i in range(9):
            for j in range(9):
                brush = QtGui.QBrush()
                brush.setStyle(Qt.SolidPattern)
                painter.setPen(QtGui.QPen(Qt.black, 3, Qt.SolidLine))
                text = ""
                if i == 0 and j == 0:
                    brush.setColor(QtGui.QColor(greycolor))
                    painter.setBrush(brush)
                elif i == 0:
                    brush.setColor(QtGui.QColor(greycolor))
                    painter.setBrush(brush)
                    text = str(9-j)
                elif j == 0:
                    brush.setColor(QtGui.QColor(greycolor))
                    painter.setBrush(brush)
                    text = chr(96+i)
                elif (j + i) % 2 == 0:
                    brush.setColor(QtGui.QColor(whitecolor))
                    painter.setBrush(brush)
                else:
                    brush.setColor(QtGui.QColor(redcolor))
                    painter.setBrush(brush)

                painter.drawRects(
                    QtCore.QRect((width * i) + 100, (height * j) + 100, width, height),
                )
                if text != "":
                    self.drawText(painter, text, (width * i) + 100, (height * j) + 100)
        painter.end()

    def drawText(self, pen, text, x, y):
        pen.setFont(QtGui.QFont("Arial", 18))
        pen.drawText(x+30, y+50, text)

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(0, 0, self.width, self.height)

        self.inputbox = QLineEdit(self)
        self.inputbox.move(900, 100)
        self.inputbox.resize(150, 30)

        self.setStyleSheet("background-color: #FCFFE9;")

    def draw_piece(self):
        #test = QtGui.QImage('src/resources/WhiteBishop.png')
        label = QtWidgets.QLabel(self)
        label.setGeometry(60, 60, 60, 60)
        label.setStyleSheet("background-color: rgba(0,0,0,0%)")
        label.setPixmap(self.whiteBishopImg)
        label.move(600, 250)

        # grid = QGridLayout()
        # for i in range(0,5):
        #     for j in range(0,5):
        #         label = QtWidgets.QLabel()
        #         label.setGeometry(50,50,50,50)
        #         label.move(50*(i+1),50*(j+1))
        #         label.setPixmap(whiteBishopImg)
        #         grid.addWidget(label)
        # self.setLayout(grid)


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


main()
