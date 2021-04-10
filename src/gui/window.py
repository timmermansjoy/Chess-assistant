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

        self.win = QtWidgets.QWidget(self)
        self.grid = QtWidgets.QGridLayout(self.win)
        self.grid.setContentsMargins(0,0,0,0)
        self.grid.setSpacing(0)        
        self.win.setLayout(self.grid)
        self.win.setGeometry(175,100,600,600)
        self.win.setStyleSheet("background-color: rgba(0,0,0,0%)")      

        self.board = Board()
        self.read_board()
            
    def read_board(self):
        for i in range(0,8):
            for j in range(0,8):
                label = QtWidgets.QLabel(self)
                label.setStyleSheet("background-color: rgba(0,0,0,0%);")
                pixmap = self.readPiece(i,j)
                if pixmap != None:
                    label.setPixmap(pixmap)
                else:
                    label.setStyleSheet("background-color: red;")
                self.grid.addWidget(label,i,j)

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
        self.setStyleSheet("background-color: #FCFFE9;")

        self.inputbox = QLineEdit(self)
        self.inputbox.move(975, 260)
        self.inputbox.resize(150, 30)
        self.inputbox.editingFinished.connect(self.enterPress)
        inputboxDescription = QtWidgets.QLabel(self)
        inputboxDescription.setText("<b>Enter your move:</b>")
        inputboxDescription.resize(150, 30)
        inputboxDescription.move(825, 260)

        cameraLabel = QtWidgets.QLabel(self)
        cameraLabel.resize(300,200)
        cameraLabel.move(830, 50)
        cameraLabel.setStyleSheet("border: 1px solid black;")
        cameraLabel.setText("<b> PLACEHOLDER CAMERA </b>")
        cameraLabel.setAlignment(QtCore.Qt.AlignCenter)

        self.movelog = QtWidgets.QLabel(self)
        self.movelog.resize(300,448)
        self.movelog.move(825, 330)
        self.movelog.setStyleSheet("border: 1px solid black;")
        self.movelog.setAlignment(QtCore.Qt.AlignLeft)
        movelogDescription = QtWidgets.QLabel(self)
        movelogDescription.setText("<b>Movelog:</b>")
        movelogDescription.resize(300,30)
        movelogDescription.move(825, 300)
        movelogDescription.setStyleSheet("border: 1px solid black;")

        errorlogDescription = QtWidgets.QLabel(self)
        errorlogDescription.setText("<b>Errorlog:</b>")
        errorlogDescription.resize(300,30)
        errorlogDescription.move(825, 800)
        errorlogDescription.setStyleSheet("border: 1px solid black;")
        self.errorlog = QtWidgets.QLabel(self)
        self.errorlog.resize(300,100)
        self.errorlog.move(825, 830)
        self.errorlog.setStyleSheet("border: 1px solid black;"
                                    "color: red;"
                                    "font-weight: bold;")
        self.errorlog.setAlignment(QtCore.Qt.AlignLeft)
        self.errorlog.setWordWrap(True)

        castleWKButton = QtWidgets.QPushButton(self)
        castleWKButton.clicked.connect(self.WKCastle)
        castleWKButton.move(475, 880)
        castleWKButton.setStyleSheet("background-color: #BEBEBE;"
                                    "font-weight: bold;")
        castleWKButton.setText("White King-side castle")
        castleWKButton.resize(225, 50)

        castleWQButton = QtWidgets.QPushButton(self)
        castleWQButton.clicked.connect(self.WQCastle)
        castleWQButton.move(175, 880)
        castleWQButton.setStyleSheet("background-color: #BEBEBE;"
                                     "font-weight: bold;")
        castleWQButton.setText("White Queen-side castle")
        castleWQButton.resize(225, 50)

        castleBKButton = QtWidgets.QPushButton(self)
        castleBKButton.clicked.connect(self.BKCastle)
        castleBKButton.move(475, 805)
        castleBKButton.setStyleSheet("background-color: #BEBEBE;"
                                    "font-weight: bold;")
        castleBKButton.setText("Black King-side castle")
        castleBKButton.resize(225, 50)

        castleBQButton = QtWidgets.QPushButton(self)
        castleBQButton.clicked.connect(self.BQCastle)
        castleBQButton.move(175, 805)
        castleBQButton.setStyleSheet("background-color: #BEBEBE;"
                                    "font-weight: bold;")
        castleBQButton.setText("Black Queen-side castle")
        castleBQButton.resize(225, 50)

        resignButton = QtWidgets.QPushButton(self)
        resignButton.clicked.connect(self.resign)
        resignButton.move(175, 25)
        resignButton.setText("Resign")
        resignButton.setStyleSheet("background-color: #CD5C5C;"
                                "font-weight: bold;")
        resignButton.resize(150, 50)

        drawButton = QtWidgets.QPushButton(self)
        drawButton.clicked.connect(self.draw)
        drawButton.move(400, 25)
        drawButton.setText("Offer draw")
        drawButton.setStyleSheet("background-color: #CD5C5C;"
                                "font-weight: bold;")
        drawButton.resize(150, 50)

        newGameButton = QtWidgets.QPushButton(self)
        newGameButton.clicked.connect(self.newGame)
        newGameButton.move(625, 25)
        newGameButton.setText("Start new game")
        newGameButton.setStyleSheet("background-color: #CD5C5C;"
                                "font-weight: bold;")
        newGameButton.resize(150, 50)

    def enterPress(self):
        try:
            inputString = str(self.inputbox.text())
            coords = self.board.notationToCords(inputString)
            self.updateBoard(coords[0].row, coords[0].column, coords[1].row, coords[1].column)
            self.board.move(coords[0].row, coords[0].column, coords[1].row, coords[1].column)
            self.updateMovelog()
            self.inputbox.clear()
        except Exception as ex:
            self.errorlog.setText(str(ex))

    def updateMovelog(self):
        self.errorlog.clear()
        self.movelog.clear()
        text = self.board.GetChessNotation()
        self.movelog.setText(text)

    def updateBoard(self, oldRow, oldColumn, newRow, newColumn):
        #delete old piece at old position
        self.grid.itemAtPosition(oldRow, oldColumn).widget().deleteLater()
        #place empty label at old piece position
        replacementLabel = QtWidgets.QLabel(self)
        replacementLabel.setStyleSheet("background-color: red;")
        self.grid.addWidget(replacementLabel, int(oldRow) ,int(oldColumn))

        #delete old piece at new position
        self.grid.itemAtPosition(newRow, newColumn).widget().deleteLater()
        #create new piece at new position
        label = QtWidgets.QLabel(self)
        label.setStyleSheet("background-color: rgba(0,0,0,0%)")
        pixmap = self.readPiece(oldRow, oldColumn)
        label.setPixmap(pixmap)
        self.grid.addWidget(label, int(newRow) ,int(newColumn))

    def WKCastle(self):
        try:
            self.board.castling(True, False, self.board.board)
        except Exception as ex:
            self.errorlog.setText(str(ex))

    def WQCastle(self):
        try:
            self.board.castling(True, True, self.board.board)
        except Exception as ex:
            self.errorlog.setText(str(ex))

    def BKCastle(self):
        try:
            self.board.castling(False, False, self.board.board)
        except Exception as ex:
            self.errorlog.setText(str(ex))

    def BQCastle(self):
        try:
            self.board.castling(False, True, self.board.board)
        except Exception as ex:
            self.errorlog.setText(str(ex))

    def resign(self):
        sys.exit()

    def draw(self):
        #TODO misschien AI laten beslissen of het wel/niet de draw accepteert
        sys.exit()

    def newGame(self):
        self.errorlog.clear()
        self.movelog.clear()
        self.clearGui()
        self.board = Board()
        self.read_board()
    
    def clearGui(self):
        for i in range(0, self.grid.rowCount()):
            for j in range(0, self.grid.columnCount()):
                self.grid.itemAtPosition(i, j).widget().deleteLater()


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setFont(QtGui.QFont("Arial", 12))
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

main()
