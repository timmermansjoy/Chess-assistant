import sys
from PyQt5 import QtCore, QtGui, QtWidgets, uic, QtSvg
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QLineEdit, QGridLayout
from PyQt5.QtGui import QPixmap, QKeyEvent
from board import Board
from testboards import Testboards as TB


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.title = 'ChessGUI'
        self.height = 1000
        self.width = 1200
        self.left = 15
        self.top = 15
        self.highlightedMove = [0, 0, 0, 0]

        self.initUI()

        self.whiteBishopImg = QPixmap('src/resources/WhiteBishop.png').scaled(73, 73, Qt.KeepAspectRatio)
        self.blackBishopImg = QPixmap('src/resources/BlackBishop.png').scaled(73, 73, Qt.KeepAspectRatio)
        self.whiteRookImg = QPixmap('src/resources/WhiteRook.png').scaled(73, 73, Qt.KeepAspectRatio)
        self.blackRookImg = QPixmap('src/resources/BlackRook.png').scaled(73, 73, Qt.KeepAspectRatio)
        self.whiteKnightImg = QPixmap('src/resources/WhiteKnight.png').scaled(73, 73, Qt.KeepAspectRatio)
        self.blackKnightImg = QPixmap('src/resources/BlackKnight.png').scaled(73, 73, Qt.KeepAspectRatio)
        self.whitePawnImg = QPixmap('src/resources/WhitePawn.png').scaled(73, 73, Qt.KeepAspectRatio)
        self.blackPawnImg = QPixmap('src/resources/BlackPawn.png').scaled(73, 73, Qt.KeepAspectRatio)
        self.whiteKingImg = QPixmap('src/resources/WhiteKing.png').scaled(73, 73, Qt.KeepAspectRatio)
        self.blackKingImg = QPixmap('src/resources/BlackKing.png').scaled(73, 73, Qt.KeepAspectRatio)
        self.whiteQueenImg = QPixmap('src/resources/WhiteQueen.png').scaled(73, 73, Qt.KeepAspectRatio)
        self.blackQueenImg = QPixmap('src/resources/BlackQueen.png').scaled(73, 73, Qt.KeepAspectRatio)

        self.win = QtWidgets.QWidget(self)
        self.grid = QtWidgets.QGridLayout(self.win)
        self.grid.setContentsMargins(0, 0, 0, 0)
        self.grid.setSpacing(0)
        self.win.setLayout(self.grid)
        self.win.setGeometry(100, 100, 675, 675)

        self.board = Board()
        self.draw_board()

    def draw_board(self):
        for i in range(0, 9):
            for j in range(0, 9):
                label = self.generate_label(i, j)
                if j != 0 and i != 8:
                    pixmap = self.readPiece(i, j-1)
                    if pixmap != None:
                        label.setPixmap(pixmap)
                self.grid.addWidget(label, i, j)

    def generate_label(self, i, j, highlightTile=False):
        label = QtWidgets.QLabel(self)
        if i == 8 and j == 0:
            label.setStyleSheet("background-color: #A4A2B8;"
                                "border: 1px solid black;")
        elif i == 8:
            label.setText(chr(96+j))
            label.setAlignment(QtCore.Qt.AlignCenter)
            label.setStyleSheet("background-color: #A4A2B8;"
                                "font-weight: bold;"
                                "font-family: Arial;"
                                "font-size: 22px;"
                                "border: 1px solid black;")
        elif j == 0:
            label.setText(str(8-i))
            label.setAlignment(QtCore.Qt.AlignCenter)
            label.setStyleSheet("background-color: #A4A2B8;"
                                "font-weight: bold;"
                                "font-family: Arial;"
                                "font-size: 22px;"
                                "border: 1px solid black;")
        elif (j + i) % 2 == 0 and highlightTile == False:
            label.setStyleSheet("background-color: #6495ED;"  # dark tile
                                "border: 1px solid black;")
        elif highlightTile == False:
            label.setStyleSheet("background-color: #CCCCFF;"  # light tile
                                "border: 1px solid black;")
        elif (j + i) % 2 == 0 and highlightTile == True:
            label.setStyleSheet("background-color: #40E0D0;"  # highlighted dark tile
                                "border: 1px solid black;")
        else:
            label.setStyleSheet("background-color: #9FE2BF;"  # highlighted light tile
                                "border: 1px solid black;")
        return label

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

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(0, 0, self.width, self.height)
        self.setStyleSheet("background-color: #adcbe3;")

        self.inputbox = QLineEdit(self)
        self.inputbox.move(975, 260)
        self.inputbox.resize(150, 30)
        # self.inputbox.connect(self.enterPress)
        self.inputbox.setStyleSheet("background-color: #FFECF5;")
        inputboxDescription = QtWidgets.QLabel(self)
        inputboxDescription.setText("<b>Enter your move:</b>")
        inputboxDescription.resize(150, 30)
        inputboxDescription.move(825, 260)

        cameraLabel = QtWidgets.QLabel(self)
        cameraLabel.resize(300, 200)
        cameraLabel.move(830, 50)
        cameraLabel.setStyleSheet("border: 1px solid black;"
                                  "background-color: #FFECF5;")
        cameraLabel.setText("<b> PLACEHOLDER CAMERA </b>")
        cameraLabel.setAlignment(QtCore.Qt.AlignCenter)

        self.movelog = QtWidgets.QTextEdit(self)
        self.movelog.setReadOnly(True)
        self.movelog.resize(300, 448)
        self.movelog.move(825, 330)
        self.movelog.setStyleSheet("border: 1px solid black;"
                                   "background-color: #FFECF5;")
        self.movelog.setAlignment(QtCore.Qt.AlignLeft)
        movelogDescription = QtWidgets.QLabel(self)
        movelogDescription.setText("<b>Movelog:</b>")
        movelogDescription.resize(300, 30)
        movelogDescription.move(825, 300)
        movelogDescription.setStyleSheet("border: 1px solid black;"
                                         "background-color: #FFECF5;")

        errorlogDescription = QtWidgets.QLabel(self)
        errorlogDescription.setText("<b>Errorlog:</b>")
        errorlogDescription.resize(300, 30)
        errorlogDescription.move(825, 800)
        errorlogDescription.setStyleSheet("border: 1px solid black;"
                                          "background-color: #FFECF5;")
        self.errorlog = QtWidgets.QLabel(self)
        self.errorlog.resize(300, 100)
        self.errorlog.move(825, 830)
        self.errorlog.setStyleSheet("border: 1px solid black;"
                                    "color: red;"
                                    "font-weight: bold;"
                                    "background-color: #FFECF5;")
        self.errorlog.setAlignment(QtCore.Qt.AlignLeft)
        self.errorlog.setWordWrap(True)

        castleWKButton = QtWidgets.QPushButton(self)
        castleWKButton.clicked.connect(self.WKCastle)
        castleWKButton.move(550, 880)
        castleWKButton.setStyleSheet("background-color: #BEBEBE;"
                                     "font-weight: bold;")
        castleWKButton.setText("White King-side castle")
        castleWKButton.resize(225, 50)

        castleWQButton = QtWidgets.QPushButton(self)
        castleWQButton.clicked.connect(self.WQCastle)
        castleWQButton.move(250, 880)
        castleWQButton.setStyleSheet("background-color: #BEBEBE;"
                                     "font-weight: bold;")
        castleWQButton.setText("White Queen-side castle")
        castleWQButton.resize(225, 50)

        castleBKButton = QtWidgets.QPushButton(self)
        castleBKButton.clicked.connect(self.BKCastle)
        castleBKButton.move(550, 805)
        castleBKButton.setStyleSheet("background-color: #BEBEBE;"
                                     "font-weight: bold;")
        castleBKButton.setText("Black King-side castle")
        castleBKButton.resize(225, 50)

        castleBQButton = QtWidgets.QPushButton(self)
        castleBQButton.clicked.connect(self.BQCastle)
        castleBQButton.move(250, 805)
        castleBQButton.setStyleSheet("background-color: #BEBEBE;"
                                     "font-weight: bold;")
        castleBQButton.setText("Black Queen-side castle")
        castleBQButton.resize(225, 50)

        resignButton = QtWidgets.QPushButton(self)
        resignButton.clicked.connect(self.resign)
        resignButton.move(175, 25)
        resignButton.setText("Resign")
        resignButton.setStyleSheet("background-color: #ffd3b6;"
                                   "font-weight: bold;")
        resignButton.resize(150, 50)

        drawButton = QtWidgets.QPushButton(self)
        drawButton.clicked.connect(self.draw)
        drawButton.move(400, 25)
        drawButton.setText("Offer draw")
        drawButton.setStyleSheet("background-color: #ffd3b6;"
                                 "font-weight: bold;")
        drawButton.resize(150, 50)

        newGameButton = QtWidgets.QPushButton(self)
        newGameButton.clicked.connect(self.newGame)
        newGameButton.move(625, 25)
        newGameButton.setText("Start new game")
        newGameButton.setStyleSheet("background-color: #ffd3b6;"
                                    "font-weight: bold;")
        newGameButton.resize(150, 50)

        invisibleButton = QtWidgets.QPushButton(self)
        invisibleButton.resize(0, 0)
        invisibleButton.setShortcut(Qt.Key_Return)
        invisibleButton.clicked.connect(self.enterPress)

        self.combobox = QtWidgets.QComboBox(self)
        self.combobox.addItems(["Queen", "Bishop", "Rook", "Knight"])
        self.combobox.setStyleSheet("background-color: #FFECF5;")
        self.combobox.move(80, 855)
        self.combobox.resize(140, 30)
        self.combobox.currentIndexChanged.connect(self.getComboboxItem)

        comboboxDescription = QtWidgets.QLabel(self)
        comboboxDescription.setText("<b>Promote pawn to:</b>")
        comboboxDescription.resize(140, 20)
        comboboxDescription.move(80, 835)

    def getComboboxItem(self):
        text = self.combobox.currentText()[0]
        if text == "K":
            text = "N"
        self.board.promotionPiece = text

    def enterPress(self):
        inputString = str(self.inputbox.text())
        error = False
        if inputString != "":
            try:
                coords = self.board.notationToCords(inputString)
                self.board.move(coords[0].row, coords[0].column, coords[1].row, coords[1].column)
                self.updateMovelog()
                self.inputbox.clear()
            except Exception as ex:
                self.errorlog.setText(str(ex))
                error = True
            if error == False:
                self.updateBoard(coords[0].row, coords[0].column, coords[1].row, coords[1].column)
        else:
            self.errorlog.setText("Input field is empty")

    def updateMovelog(self):
        self.errorlog.clear()
        self.movelog.clear()
        text = self.board.GetChessNotation()
        self.movelog.setText(text)

    def updateBoard(self, oldRow, oldColumn, newRow, newColumn):
        # The arguments of this function are of an 8x8 array, but they are used in a 9x9 array
        # +1 is added to the columns when they are used in functions because column 0 is filled with labels that don't contain chess pieces
        # Remove old highlighted tiles
        if self.highlightedMove != [0, 0, 0, 0]:
            self.grid.itemAtPosition(self.highlightedMove[0], self.highlightedMove[1]+1).widget().deleteLater()
            replacementLabel = self.generate_label(int(self.highlightedMove[0]), int(self.highlightedMove[1]+1), False)
            self.grid.addWidget(replacementLabel, int(self.highlightedMove[0]), int(self.highlightedMove[1]+1))

            self.grid.itemAtPosition(self.highlightedMove[2], self.highlightedMove[3]+1).widget().deleteLater()
            label = self.generate_label(int(self.highlightedMove[2]), int(self.highlightedMove[3]+1), False)
            pixmap = self.readPiece(int(self.highlightedMove[2]), int(self.highlightedMove[3]))
            label.setPixmap(pixmap)
            self.grid.addWidget(label, int(self.highlightedMove[2]), int(self.highlightedMove[3]+1))

        # delete old piece at old position
        self.grid.itemAtPosition(oldRow, oldColumn+1).widget().deleteLater()
        # place empty label at old piece position
        replacementLabel = self.generate_label(int(oldRow), int(oldColumn+1), True)
        self.grid.addWidget(replacementLabel, int(oldRow), int(oldColumn+1))

        # delete old piece at new position
        self.grid.itemAtPosition(newRow, newColumn+1).widget().deleteLater()
        # create new piece at new position
        label = self.generate_label(int(newRow), int(newColumn+1), True)
        pixmap = self.readPiece(newRow, newColumn)
        label.setPixmap(pixmap)
        self.grid.addWidget(label, int(newRow), int(newColumn+1))
        self.highlightedMove = [oldRow, oldColumn, newRow, newColumn]

    def WKCastle(self):
        try:
            self.board.castling(True, False, self.board.board)
            self.updateBoard(7, 4, 7, 6)
            self.updateBoard(7, 7, 7, 5)
        except Exception as ex:
            self.errorlog.setText(str(ex))

    def WQCastle(self):
        try:
            self.board.castling(True, True, self.board.board)
            self.updateBoard(7, 4, 7, 2)
            self.updateBoard(7, 0, 7, 3)
        except Exception as ex:
            self.errorlog.setText(str(ex))

    def BKCastle(self):
        try:
            self.board.castling(False, False, self.board.board)
            self.updateBoard(0, 4, 0, 6)
            self.updateBoard(0, 7, 0, 5)
        except Exception as ex:
            self.errorlog.setText(str(ex))

    def BQCastle(self):
        try:
            self.board.castling(False, True, self.board.board)
            self.updateBoard(0, 4, 0, 2)
            self.updateBoard(0, 0, 0, 3)
        except Exception as ex:
            self.errorlog.setText(str(ex))

    def resign(self):
        sys.exit()

    def draw(self):
        # TODO misschien AI laten beslissen of het wel/niet de draw accepteert
        sys.exit()

    def newGame(self):
        self.errorlog.clear()
        self.movelog.clear()
        self.clearGui()
        self.board = Board()
        self.draw_board()

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


if __name__ == "__main__":
    main()
