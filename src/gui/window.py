
import sys
from PyQt5 import QtCore, QtGui, QtWidgets, uic, QtSvg
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtGui import QPixmap

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.title = 'ChessGUI'
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
                elif (j + i) % 2 == 1:               
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
        whiteBishopImg = QPixmap('src/resources/WhiteBishop.png')
        whiteBishopImg.scaled(50 , 50)
        
        #whiteBishopImg = pygame.transform.scale(whiteBishopImg, (width, width))
        blackBishopImg = QPixmap('src/resources/BlackBishop.png')
        #blackBishopImg = pygame.transform.scale(blackBishopImg, (width, width))
        whiteRookImg = QPixmap('src/resources/WhiteRook.png')
        #whiteRookImg = pygame.transform.scale(whiteRookImg, (width, width))
        blackRookImg = QPixmap('src/resources/BlackRook.png')
        #blackRookImg = pygame.transform.scale(blackRookImg, (width, width))
        whiteKnightImg = QPixmap('src/resources/WhiteKnight.png')
        #whiteKnightImg = pygame.transform.scale(whiteKnightImg, (width, width))
        blackKnightImg = QPixmap('src/resources/BlackKnight.png')
        #blackKnightImg = pygame.transform.scale(blackKnightImg, (width, width))
        whitePawnImg = QPixmap('src/resources/WhitePawn.png')
        #whitePawnImg = pygame.transform.scale(whitePawnImg, (width, width))
        blackPawnImg = QPixmap('src/resources/BlackPawn.png')
        #blackPawnImg = pygame.transform.scale(blackPawnImg, (width, width))
        whiteKingImg = QPixmap('src/resources/WhiteKing.png')
        #whiteKingImg = pygame.transform.scale(whiteKingImg, (width, width))
        blackKingImg = QPixmap('src/resources/BlackKing.png')
        #blackKingImg = pygame.transform.scale(blackKingImg, (width, width))
        whiteQueenImg = QPixmap('src/resources/WhiteQueen.png')
        #whiteQueenImg = pygame.transform.scale(whiteQueenImg, (width, width))
        blackQueenImg = QPixmap('src/resources/BlackQueen.png')
        #blackQueenImg = pygame.transform.scale(blackQueenImg, (width, width))


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

main()
