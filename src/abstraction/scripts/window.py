#!/usr/bin/env python3
import sys
from PyQt5 import QtCore, QtGui, QtWidgets, uic, QtSvg
from PyQt5.QtCore import QObject, Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtWidgets import QLineEdit, QGridLayout, QMessageBox
from PyQt5.QtGui import QPixmap, QKeyEvent
from board import Board
from testboards import Testboards as TB
from extra import Coordinate
import os

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from threading import Lock
import cv2
from cv_bridge import CvBridge
import numpy as np
import ai

playvsAi = False
suggestMove = False
playOnVision = False
board = Board()
class Worker(QObject):
    finished = pyqtSignal(Coordinate, Coordinate)

    def run(self):
        print("Computers Turn:")
        beginCoord, endCoord = ai.calculateMove(3, board, False)
        self.finished.emit(beginCoord, endCoord)



class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.title = 'ChessGUI'
        self.height = 1000
        self.width = 1200
        self.left = 15
        self.top = 15
        self.highlightedMove = [0, 0, 0, 0]
        self.bridge = CvBridge()
        self.init_subscriber()
        self.GUI_UPDATE_PERIOD = 10
        self.initUI()

        self.whiteBishopImg = QPixmap(self.find_image("WhiteBishop.png")).scaled(73, 73, Qt.KeepAspectRatio)
        self.blackBishopImg = QPixmap(self.find_image("BlackBishop.png")).scaled(73, 73, Qt.KeepAspectRatio)
        self.whiteRookImg = QPixmap(self.find_image("WhiteRook.png")).scaled(73, 73, Qt.KeepAspectRatio)
        self.blackRookImg = QPixmap(self.find_image("BlackRook.png")).scaled(73, 73, Qt.KeepAspectRatio)
        self.whiteKnightImg = QPixmap(self.find_image('WhiteKnight.png')).scaled(73, 73, Qt.KeepAspectRatio)
        self.blackKnightImg = QPixmap(self.find_image('BlackKnight.png')).scaled(73, 73, Qt.KeepAspectRatio)
        self.whitePawnImg = QPixmap(self.find_image('WhitePawn.png')).scaled(73, 73, Qt.KeepAspectRatio)
        self.blackPawnImg = QPixmap(self.find_image('BlackPawn.png')).scaled(73, 73, Qt.KeepAspectRatio)
        self.whiteKingImg = QPixmap(self.find_image('WhiteKing.png')).scaled(73, 73, Qt.KeepAspectRatio)
        self.blackKingImg = QPixmap(self.find_image('BlackKing.png')).scaled(73, 73, Qt.KeepAspectRatio)
        self.whiteQueenImg = QPixmap(self.find_image('WhiteQueen.png')).scaled(73, 73, Qt.KeepAspectRatio)
        self.blackQueenImg = QPixmap(self.find_image('BlackQueen.png')).scaled(73, 73, Qt.KeepAspectRatio)

        self.win = QtWidgets.QWidget(self)
        self.grid = QtWidgets.QGridLayout(self.win)
        self.grid.setContentsMargins(0, 0, 0, 0)
        self.grid.setSpacing(0)
        self.win.setLayout(self.grid)
        self.win.setGeometry(100, 100, 675, 675)

        global board
        #self.previousBoard = Board()
        self.draw_board()

        self.isInCheckmate = False

    # when receiving a board from abstraction, put it in board and trigger this to get the fields that should get highlighted.
    # def difference_in_boards():
    #     changedFields = []
    #     for i in range (8):
    #         for j in range (8):
    #             if board.board[i][j] != self.previousBoard.board[i][j]:
    #                 changedFields.append([i,j])
    #     self.previousBoard = board
    #     return changedFields

    def find_image(self, image_name):
        path = os.path.realpath("")
        for root, dirs, files in os.walk(path):
            if image_name in files:
                return os.path.join(root, image_name)
        return None

    def init_subscriber(self):
        self.ros_image_lock = Lock()

        self.ros_image_lock.acquire()
        try:
            self.received_new_data = False
        finally:
            self.ros_image_lock.release()

        # Start to listen...
        self.subscriber = rospy.Subscriber("/chesscam/compressed", Image, self.callback_image_raw)
        rospy.loginfo('subscribed to topic /chesscam/compressed')

        self.move_sub = rospy.Subscriber('visionMove', String, self.playOnVisionSubscriber)
        rospy.loginfo('subscribed to visionMove')

    def callback_image_raw(self, image):
        self.ros_image_lock.acquire()
        try:
            self.ros_image = image
            self.received_new_data = True
        finally:
            self.ros_image_lock.release()

    def update_image_on_gui(self):
        # Get a new image if there's one and make a copy of it.
        new_image = False
        self.ros_image_lock.acquire()
        try:
            if self.received_new_data == True:
                new_image = True
                opencv_image = self.convert_ros_to_opencv(self.ros_image)
                self.received_new_data = False
        finally:
            self.ros_image_lock.release()

        if not new_image:
            return

        scale = 0.4
        interpolation = cv2.INTER_AREA
        width = int(opencv_image.shape[1] * scale)
        height = int(opencv_image.shape[0] * scale)
        dimensions = (width, height)

        scaled_image = cv2.resize(opencv_image, dimensions, interpolation)

        # Conver the scaled image to a QImage and show it on the GUI.
        rgb_image = cv2.cvtColor(scaled_image, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb_image.shape
        bytes_per_line = channels * width
        qt_image = QtGui.QImage(
            rgb_image.data, width, height, bytes_per_line, QtGui.QImage.Format_RGB888)
        self.cameraLabel.setPixmap(QPixmap.fromImage(qt_image))

    def convert_ros_to_opencv(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return cv_image
        except CvBridgeError as error:
            raise Exception("Failed to convert to OpenCV image")

    def draw_board(self):
        for i in range(0, 9):
            for j in range(0, 9):
                label = self.generate_label(i, j)
                if j != 0 and i != 8:
                    pixmap = self.readPiece(i, j-1)
                    if pixmap != None:
                        label.setPixmap(pixmap)
                        label.update()
                    else:
                        label.update()
                self.grid.addWidget(label, i, j)
        self.update()

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
        currentPiece = board.board[i][j]
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
        self.inputbox.setStyleSheet("background-color: #FFECF5; color: #000000")
        inputboxDescription = QtWidgets.QLabel(self)
        inputboxDescription.setText("<b>Enter your move:</b>")
        inputboxDescription.resize(150, 30)
        inputboxDescription.move(825, 260)

        self.cameraLabel = QtWidgets.QLabel(self)
        self.cameraLabel.resize(300, 200)
        self.cameraLabel.move(830, 50)
        self.cameraLabel.setStyleSheet("border: 1px solid black;"
                                       "background-color: #FFECF5; color: #000000")
        #self.cameraLabel.setText("<b> PLACEHOLDER CAMERA </b>")
        self.cameraLabel.setAlignment(QtCore.Qt.AlignCenter)

        self.movelog = QtWidgets.QTextEdit(self)
        self.movelog.setReadOnly(True)
        self.movelog.resize(300, 448)
        self.movelog.move(825, 330)
        self.movelog.setStyleSheet("border: 1px solid black;"
                                   "background-color: #FFECF5; color: #000000")
        self.movelog.setAlignment(QtCore.Qt.AlignLeft)
        movelogDescription = QtWidgets.QLabel(self)
        movelogDescription.setText("<b>Movelog:</b>")
        movelogDescription.resize(300, 30)
        movelogDescription.move(825, 300)
        movelogDescription.setStyleSheet("border: 1px solid black;"
                                         "background-color: #FFECF5; color: #000000")

        errorlogDescription = QtWidgets.QLabel(self)
        errorlogDescription.setText("<b>Errorlog:</b>")
        errorlogDescription.resize(300, 30)
        errorlogDescription.move(825, 800)
        errorlogDescription.setStyleSheet("border: 1px solid black;"
                                          "background-color: #FFECF5; color: #000000")
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

        self.invisibleButton = QtWidgets.QPushButton(self)
        self.invisibleButton.resize(0, 0)
        self.invisibleButton.setShortcut(Qt.Key_Return)
        self.invisibleButton.clicked.connect(self.enterPress)

        self.combobox = QtWidgets.QComboBox(self)
        self.combobox.addItems(["Queen", "Bishop", "Rook", "Knight"])
        self.combobox.setStyleSheet("background-color: #FFECF5; color: #000000")
        self.combobox.move(80, 855)
        self.combobox.resize(140, 30)
        self.combobox.currentIndexChanged.connect(self.getComboboxItem)

        comboboxDescription = QtWidgets.QLabel(self)
        comboboxDescription.setText("<b>Promote pawn to:</b>")
        comboboxDescription.resize(140, 20)
        comboboxDescription.move(80, 835)

        # Start to update the image on the gui.
        self.gui_timer = QTimer(self)
        self.gui_timer.start(self.GUI_UPDATE_PERIOD)
        self.gui_timer.timeout.connect(self.update_image_on_gui)

        self.messageBox = QMessageBox(self)
        self.messageBox.resize(400, 300)

        if playOnVision:
            self.inputbox.resize(0,0)
            inputboxDescription.resize(0,0)
            castleWKButton.resize(0,0)
            castleWQButton.resize(0,0)
            castleBQButton.resize(0,0)
            castleBKButton.resize(0,0)
            self.combobox.resize(0,0)
            comboboxDescription.resize(0,0)

    def getComboboxItem(self):
        text = self.combobox.currentText()[0]
        if text == "K":
            text = "N"
        board.promotionPiece = text

    def playOnVisionSubscriber(self, rawmsg):
        error=False
        try:
            msg = rawmsg.data
            currentMoveIsCheck = board.isCheck(board.isWhitePlayerTurn, int(msg[1]), int(msg[4]), int(msg[7]), int(msg[10]))
            board.move(int(msg[1]), int(msg[4]), int(msg[7]), int(msg[10]))
            print(int(msg[1]), int(msg[4]), int(msg[7]), int(msg[10]))
        except Exception as ex:
            if('is not a valid move' in str(ex)):
                try:
                    msg = rawmsg.data

                    board.move(int(msg[7]), int(msg[10]), int(msg[1]), int(msg[4]))
                except Exception as ex2:
                    self.errorlog.setText(str(ex2))
                    print("ex2 thrown")
                    error = True
            else:
                self.errorlog.setText(str(ex))
                error = True
        print("move finished")

        if error == False:
            self.updateMovelog()
            self.clearGui()
            self.draw_board()
            self.checkmateCheck()
            try:
                if suggestMove == True:
                    self.aiMoveOrSuggest()
            except Exception as ex:
                self.errorlog.setText(str(ex))
            if currentMoveIsCheck:
                self.colorKingField(1)
        
    def enterPress(self):
        print(board.board)
        inputString = str(self.inputbox.text())
        error = False
        currentMoveIsCheck = False
        if inputString != "":
            try:
                coords = board.notationToCords(inputString)
                currentMoveIsCheck = board.isCheck(board.isWhitePlayerTurn, coords[0].row, coords[0].column, coords[1].row, coords[1].column)
                board.move(coords[0].row, coords[0].column, coords[1].row, coords[1].column)
                self.inputbox.clear()
            except Exception as ex:
                self.errorlog.setText(str(ex))
                error = True
            if error == False:
                self.updateMovelog()
                self.clearGui()
                self.draw_board()
                self.checkmateCheck()
                print("redraw")
                if not self.isInCheckmate:
                    if playvsAi == True:
                        self.aiMoveOrSuggest(True)
                    try:
                        if suggestMove == True and playvsAi==False:
                            self.aiMoveOrSuggest()
                    except Exception as ex:
                        self.errorlog.setText(str(ex))
                    if currentMoveIsCheck and not playvsAi:
                        self.colorKingField(1)
        else:
            self.errorlog.setText("Input field is empty")

    def moveFromCoordinates(self, coordinate1, coordinate2):
        currentMoveIsCheck = board.isCheck(board.isWhitePlayerTurn, coordinate1.row, coordinate1.column, coordinate2.row, coordinate2.column)
        board.move(coordinate1.row, coordinate1.column, coordinate2.row, coordinate2.column)
        self.updateMovelog()
        self.clearGui()
        self.draw_board()
        if(currentMoveIsCheck):
            self.colorKingField(1)
        if suggestMove == True:
            self.aiMoveOrSuggest()

    def aiMoveOrSuggest(self, value=False):
        self.inputbox.setStyleSheet("background-color: grey;")
        self.inputbox.setEnabled(False)
        self.errorlog.setText("your move has been made, awaiting a computer generated move.")
        if value:
            self.messageBox.setText("Your move was made, please wait patiently for the AI")
        else:
            self.messageBox.setText("the AI is generating a proposed move, please wait")
        self.messageBox.exec()
        self.worker = Worker()
        self.thread = QThread()
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)
        bonusString = "you can now make your own move"
        if suggestMove:
            bonusString = "Please wait for the computer to suggest your next move"
        if value==True:
            self.worker.finished.connect(self.moveFromCoordinates)
            self.thread.finished.connect(
            lambda: self.errorlog.setText("the AI made it's move!\n" + bonusString)
        )
        else:
            self.worker.finished.connect(self.suggestMove)
            self.thread.finished.connect(
            lambda: self.errorlog.setText("move has been generated. You can now see the suggested move.")
        )
        self.thread.start()
        self.inputbox.setEnabled(True)
        self.inputbox.setStyleSheet("background-color: #FFECF5;")


    def suggestMove(self, beginCoord, endCoord):
        if not (beginCoord.row == 0 and beginCoord.column ==0 and endCoord.row==0 and endCoord.column==0):
            self.highlightSuggestedMove(beginCoord.row, beginCoord.column, endCoord.row, endCoord.column)

    def updateMovelog(self):
        self.errorlog.clear()
        self.movelog.clear()
        text = board.GetChessNotation()
        self.movelog.setText(text)

    def highlightMove(self, oldRow, oldColumn, newRow, newColumn):
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

    def highlightSuggestedMove(self, oldRow, oldColumn, newRow, newColumn):
        # delete old piece at old position
        self.grid.itemAtPosition(oldRow, oldColumn+1).widget().deleteLater()
        # place empty label at old piece position
        replacementLabel = self.generate_label(int(oldRow), int(oldColumn+1), True)
        pixmap = self.readPiece(oldRow, oldColumn)
        replacementLabel.setPixmap(pixmap)
        self.grid.addWidget(replacementLabel, int(oldRow), int(oldColumn+1))

        # delete old piece at new position
        self.grid.itemAtPosition(newRow, newColumn+1).widget().deleteLater()
        # create new piece at new position
        label = self.generate_label(int(newRow), int(newColumn+1), True)
        self.grid.addWidget(label, int(newRow), int(newColumn+1))
        self.highlightedMove = [oldRow, oldColumn, newRow, newColumn]

    def WKCastle(self):
        try:
            board.castling(True, False, board.board)
            self.clearGui()
            self.draw_board()
            self.highlightMove(7, 4, 7, 6)
        except Exception as ex:
            self.errorlog.setText(str(ex))

    def WQCastle(self):
        try:
            board.castling(True, True, board.board)
            self.clearGui()
            self.draw_board()
            self.highlightMove(7, 4, 7, 2)
        except Exception as ex:
            self.errorlog.setText(str(ex))

    def BKCastle(self):
        try:
            board.castling(False, False, board.board)
            self.clearGui()
            self.draw_board()
            self.highlightMove(0, 4, 0, 6)
        except Exception as ex:
            self.errorlog.setText(str(ex))

    def BQCastle(self):
        try:
            board.castling(False, True, board.board)
            self.clearGui()
            self.draw_board()
            self.highlightMove(0, 4, 0, 2)
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
        self.highlightedMove = [0, 0, 0, 0]
        board = Board()
        self.draw_board()

    def clearGui(self):
        for i in range(0, self.grid.rowCount()):
            for j in range(0, self.grid.columnCount()):
                self.grid.itemAtPosition(i, j).widget().deleteLater()

    def colorKingField(self, value=0):
        king = "K" if board.isWhitePlayerTurn else "k"
        for row in range(8):
            for col in range(8):
                if board.board[row][col] == king:
                    kingRow = row
                    kingColumn = col
        self.grid.itemAtPosition(kingRow, kingColumn+1).widget().deleteLater()
        label = QtWidgets.QLabel(self)
        if value==1:
            label.setStyleSheet("background-color: #ff9500;"
                            "border: 1px solid black;")
        else:
            label.setStyleSheet("background-color: #8f0000;"
                            "border: 1px solid black;")
        pixmap = self.readPiece(kingRow, kingColumn)
        label.setPixmap(pixmap)
        self.grid.addWidget(label, int(kingRow), int(kingColumn+1))

    def checkmateCheck(self):
        print("checkmate start")
        thisPlayer = board.isWhitePlayerTurn
        self.isInCheckmate = board.isInCheckmate(thisPlayer)
        if self.isInCheckmate:
            if thisPlayer:
                thisPlayerString = "white"
                otherPlayerString = "black"
            else:
                thisPlayerString = "black"
                otherPlayerString = "white"
            returnString = "The current " + thisPlayerString + " player has won the game by putting the " + otherPlayerString + \
                " player in checkmate, the game has ended.\n \n" + "You can now start another game by restarting the app or pressing \" New game\""
            self.colorKingField()
            self.messageBox.setText(returnString)
            board.isCheckmate = True
            self.messageBox.exec()
        print("checkmate end")


class SettingsWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.title = 'Settings'
        self.setGeometry(0, 0, 600, 800)

        saveButton = QtWidgets.QPushButton(self)
        saveButton.clicked.connect(self.openMainWindow)
        saveButton.move(225, 725)
        saveButton.setText("Save settings")
        saveButton.setStyleSheet("background-color: #73A657;"
                                 "font-weight: bold;")
        saveButton.resize(150, 50)

        suggestMoveCheckbox = QtWidgets.QCheckBox("Suggest move", self)
        suggestMoveCheckbox.stateChanged.connect(self.checkSuggestMove)
        suggestMoveCheckbox.move(25, 75)
        suggestMoveCheckbox.resize(200, 50)

        playvsAiCheckbox = QtWidgets.QCheckBox("Play vs AI", self)
        playvsAiCheckbox.stateChanged.connect(self.checkPlayvsAi)
        playvsAiCheckbox.move(25, 25)
        playvsAiCheckbox.resize(200, 50)

        playOnVisionCheckbox = QtWidgets.QCheckBox("""
        Play using the input coming from the camera instead of 
        \n using the manual input. If you want to get moves
        \n suggested to you, check 'suggest moves' as well.
        \n this mode will not allow any""", self)
        playOnVisionCheckbox.stateChanged.connect(self.checkPlayOnVision)
        playOnVisionCheckbox.move(25, 125)
        playOnVisionCheckbox.resize(500, 50)

    def openMainWindow(self):
        self.close()
        window = MainWindow()
        window.show()

    def checkSuggestMove(self, state):
        global suggestMove
        if state == QtCore.Qt.Checked:
            suggestMove = True
        else:
            suggestMove = False

    def checkPlayvsAi(self, state):
        global playvsAi
        if state == QtCore.Qt.Checked:
            playvsAi = True
        else:
            playvsAi = False
    
    def checkPlayOnVision(self, state):
        global playOnVision
        if state == QtCore.Qt.Checked:
            playOnVision = True
        else:
            False


def main():
    rospy.init_node('window')
    rospy.loginfo('window node has been initialized')

    app = QtWidgets.QApplication(sys.argv)
    app.setFont(QtGui.QFont("Arial", 12))

    gamesettings = SettingsWindow()
    gamesettings.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
