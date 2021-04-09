import sys
from PyQt5 import QtCore, QtGui, QtWidgets, uic, QtSvg
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QLineEdit, QGridLayout
from PyQt5.QtGui import QPixmap
from board import Board

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.title = 'ChessGUI'

        self.whiteBishopImg = QPixmap('src/resources/WhiteBishop.png')
        self.whiteBishopImg = self.whiteBishopImg.scaled(75 , 75, Qt.KeepAspectRatio)
        self.blackBishopImg = QPixmap('src/resources/BlackBishop.png')
        self.blackBishopImg = self.blackBishopImg.scaled(75 , 75, Qt.KeepAspectRatio)
        self.whiteRookImg = QPixmap('src/resources/WhiteRook.png')
        self.whiteRookImg = self.whiteRookImg.scaled(75 , 75, Qt.KeepAspectRatio)
        self.blackRookImg = QPixmap('src/resources/BlackRook.png')
        self.blackRookImg = self.blackRookImg.scaled(75 , 75, Qt.KeepAspectRatio)
        self.whiteKnightImg = QPixmap('src/resources/WhiteKnight.png')
        self.whiteKnightImg = self.whiteKnightImg.scaled(75 , 75, Qt.KeepAspectRatio)
        self.blackKnightImg = QPixmap('src/resources/BlackKnight.png')
        self.blackKnightImg = self.blackKnightImg.scaled(75 , 75, Qt.KeepAspectRatio)
        self.whitePawnImg = QPixmap('src/resources/WhitePawn.png')
        self.whitePawnImg = self.whitePawnImg.scaled(75 , 75, Qt.KeepAspectRatio)
        self.blackPawnImg = QPixmap('src/resources/BlackPawn.png')
        self.blackPawnImg = self.blackPawnImg.scaled(75 , 75, Qt.KeepAspectRatio)
        self.whiteKingImg = QPixmap('src/resources/WhiteKing.png')
        self.whiteKingImg = self.whiteKingImg.scaled(75 , 75, Qt.KeepAspectRatio)
        self.blackKingImg = QPixmap('src/resources/BlackKing.png')
        self.blackKingImg = self.blackKingImg.scaled(75 , 75, Qt.KeepAspectRatio)
        self.whiteQueenImg = QPixmap('src/resources/WhiteQueen.png')
        self.whiteQueenImg = self.whiteQueenImg.scaled(75 , 75, Qt.KeepAspectRatio)
        self.blackQueenImg = QPixmap('src/resources/BlackQueen.png')
        self.blackQueenImg = self.blackQueenImg.scaled(75 , 75, Qt.KeepAspectRatio)

        self.board = Board()
        self.height = 1000
        self.width = 1200
        self.left = 15
        self.top = 15
        self.initUI()
        self.draw_piece()       
        self.read_board()

        gameNotOngoing = False
        #while not gameNotOngoing:
            

    def read_board(self):
            win = QtWidgets.QWidget(self)
            grid = QtWidgets.QGridLayout(win)
            
            for i in range(0,8):
                for j in range(0,8):
                    currentPiece = self.board.board[i][j]
                    label = QtWidgets.QLabel(self)
                    label.setStyleSheet("background-color: rgba(0,0,0,0%)")
                    if currentPiece == ".":
                        pass
                    elif currentPiece == "k":
                        label.setPixmap(self.blackKingImg)
                    elif currentPiece == "p":
                        label.setPixmap(self.blackPawnImg)
                    elif currentPiece == "q":
                        label.setPixmap(self.blackQueenImg)
                    elif currentPiece == "b":
                        label.setPixmap(self.blackBishopImg)
                    elif currentPiece == "n":
                        label.setPixmap(self.blackKnightImg)
                    elif currentPiece == "r":
                        label.setPixmap(self.blackRookImg)
                    elif currentPiece == "K":
                        label.setPixmap(self.whiteKingImg)
                    elif currentPiece == "P":
                        label.setPixmap(self.whitePawnImg)
                    elif currentPiece == "Q":
                        label.setPixmap(self.whiteQueenImg)
                    elif currentPiece == "B":
                        label.setPixmap(self.whiteBishopImg)
                    elif currentPiece == "N":
                        label.setPixmap(self.whiteKnightImg)
                    elif currentPiece == "R":
                        label.setPixmap(self.whiteRookImg)
                    grid.addWidget(label,i,j)

            win.setLayout(grid)
            win.setGeometry(165,90,615,625)
            win.setStyleSheet("background-color: rgba(0,0,0,0%)")


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
                if i == 0 and j == 8:                
                    brush.setColor(QtGui.QColor(greycolor))                    
                    painter.setBrush(brush)
                elif i == 0:                
                    brush.setColor(QtGui.QColor(greycolor))                    
                    painter.setBrush(brush)
                    text = str(8-j)
                elif j == 8:                
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
        self.inputbox.editingFinished.connect(self.enterPress)

        self.setStyleSheet("background-color: #FCFFE9;")

    def enterPress(self):
        inputString = str(self.inputbox.text())
        coords = self.board.notationToCords(inputString)
        self.board.move(coords[0].row, coords[0].column, coords[1].row, coords[1].column)
        self.inputbox.clear()

    #def updateBoard(self):


    def draw_piece(self):
        label = QtWidgets.QLabel(self)
        label.setGeometry(60,60,60,60)
        label.setStyleSheet("background-color: rgba(0,0,0,0%)")
        #label.setPixmap(self.whiteBishopImg)
        label.move(600, 250)


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

main()
