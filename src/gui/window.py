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
        self.height = 1000
        self.width = 1200
        self.left = 15
        self.top = 15
        self.initUI()   

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
        self.read_board()

        gameNotOngoing = False
        #while not gameNotOngoing:
            
    def read_board(self):
            win = QtWidgets.QWidget(self)
            self.grid = QtWidgets.QGridLayout(win)        
            for i in range(0,8):
                for j in range(0,8):
                    label = QtWidgets.QLabel(self)
                    label.setStyleSheet("background-color: rgba(0,0,0,0%)")
                    pixmap = self.readPiece(i,j)
                    if pixmap != None:
                        label.setPixmap(pixmap)
                    self.grid.addWidget(label,i,j)

            win.setLayout(self.grid)
            win.setGeometry(165,90,615,625)
            win.setStyleSheet("background-color: rgba(0,0,0,0%)")

    def readPiece(self, i, j):
            currentPiece = self.board.board[i][j]
            if currentPiece == ".":
                pass
            elif currentPiece == "k":
                return self.blackKingImg
            elif currentPiece == "p":
                return self.blackPawnImg
            elif currentPiece == "q":
                return self.blackQueenImg
            elif currentPiece == "b":
                return self.blackBishopImg
            elif currentPiece == "n":
                return self.blackKnightImg
            elif currentPiece == "r":
                return self.blackRookImg
            elif currentPiece == "K":
                return self.whiteKingImg
            elif currentPiece == "P":
                return self.whitePawnImg
            elif currentPiece == "Q":
                return self.whiteQueenImg
            elif currentPiece == "B":
                return self.whiteBishopImg
            elif currentPiece == "N":
                return self.whiteKnightImg
            elif currentPiece == "R":
                return self.whiteRookImg

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
        self.inputbox.move(975, 100)
        self.inputbox.resize(150, 30)
        self.inputbox.editingFinished.connect(self.enterPress)
        inputboxDescription = QtWidgets.QLabel(self)
        inputboxDescription.setText("<b>Enter your move:</b>")
        inputboxDescription.setFont(QtGui.QFont("Arial", 12))
        inputboxDescription.resize(150, 30)
        inputboxDescription.move(825, 100)

        self.movelog = QtWidgets.QLabel(self)
        self.movelog.setFont(QtGui.QFont("Arial", 12))
        self.movelog.resize(200,250)
        self.movelog.move(875, 170)
        self.movelog.setStyleSheet("border: 1px solid black;")
        self.movelog.setAlignment(QtCore.Qt.AlignLeft)
        movelogDescription = QtWidgets.QLabel(self)
        movelogDescription.setFont(QtGui.QFont("Arial", 12))
        movelogDescription.setText("<b>Movelog:</b>")
        movelogDescription.resize(200,30)
        movelogDescription.move(875, 140)
        movelogDescription.setStyleSheet("border: 1px solid black;")

        self.setStyleSheet("background-color: #FCFFE9;")

    def enterPress(self):
        inputString = str(self.inputbox.text())
        coords = self.board.notationToCords(inputString)
        self.updateBoard(coords[0].row, coords[0].column, coords[1].row, coords[1].column)
        self.updateMovelog()
        self.board.move(coords[0].row, coords[0].column, coords[1].row, coords[1].column)
        self.inputbox.clear()

    def updateMovelog(self):
        self.movelog.clear()
        text = self.board.GetChessNotation()
        self.movelog.setText(text)

    def updateBoard(self, oldRow, oldColumn, newRow, newColumn):
        #delete old piece at old position
        self.grid.itemAtPosition(oldRow, oldColumn).widget().deleteLater()
        #place empty label at old piece position
        replacementLabel = QtWidgets.QLabel(self)
        replacementLabel.setStyleSheet("background-color: rgba(0,0,0,0%);")
        self.grid.addWidget(replacementLabel, int(oldRow) ,int(oldColumn))

        #delete old piece at new position
        self.grid.itemAtPosition(newRow, newColumn).widget().deleteLater()
        #create new piece at new position
        label = QtWidgets.QLabel(self)
        label.setStyleSheet("background-color: rgba(0,0,0,0%)")
        pixmap = self.readPiece(oldRow, oldColumn)
        label.setPixmap(pixmap)
        self.grid.addWidget(label, int(newRow) ,int(newColumn))

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

main()
