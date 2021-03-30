import pygame
import traceback
from board import Board
from testboards import Testboards as TB

pygame.init()
board = Board()
board.board = TB.Castle
validChars = "12345678 abcdefgh"
shiftChars = '12345678 ABCDEFGH'
shiftDown = False

display_width = 1200
display_height = 1000

width = int(600 / 8)
height = int(600 / 8)
horizontalOffset = 50
verticalOffset = 100

gameDisplay = pygame.display.set_mode((display_width, display_height))
font = pygame.font.SysFont("Arial", 18)
pygame.display.set_caption('Ai chess')

red = (173, 91, 75)
white = (255, 255, 255)
grey = (236, 216, 194)
blue = (0, 32, 255)

clock = pygame.time.Clock()
gameNotOngoing = False
# importing pieces
whiteBishopImg = pygame.image.load('src/resources/WhiteBishop.png')
whiteBishopImg = pygame.transform.scale(whiteBishopImg, (width, width))
blackBishopImg = pygame.image.load('src/resources/BlackBishop.png')
blackBishopImg = pygame.transform.scale(blackBishopImg, (width, width))
whiteRookImg = pygame.image.load('src/resources/WhiteRook.png')
whiteRookImg = pygame.transform.scale(whiteRookImg, (width, width))
blackRookImg = pygame.image.load('src/resources/BlackRook.png')
blackRookImg = pygame.transform.scale(blackRookImg, (width, width))
whiteKnightImg = pygame.image.load('src/resources/WhiteKnight.png')
whiteKnightImg = pygame.transform.scale(whiteKnightImg, (width, width))
blackKnightImg = pygame.image.load('src/resources/BlackKnight.png')
blackKnightImg = pygame.transform.scale(blackKnightImg, (width, width))
whitePawnImg = pygame.image.load('src/resources/WhitePawn.png')
whitePawnImg = pygame.transform.scale(whitePawnImg, (width, width))
blackPawnImg = pygame.image.load('src/resources/BlackPawn.png')
blackPawnImg = pygame.transform.scale(blackPawnImg, (width, width))
whiteKingImg = pygame.image.load('src/resources/WhiteKing.png')
whiteKingImg = pygame.transform.scale(whiteKingImg, (width, width))
blackKingImg = pygame.image.load('src/resources/BlackKing.png')
blackKingImg = pygame.transform.scale(blackKingImg, (width, width))
whiteQueenImg = pygame.image.load('src/resources/WhiteQueen.png')
whiteQueenImg = pygame.transform.scale(whiteQueenImg, (width, width))
blackQueenImg = pygame.image.load('src/resources/BlackQueen.png')
blackQueenImg = pygame.transform.scale(blackQueenImg, (width, width))


def update_fps():
    fps = str(int(clock.get_fps()))
    fps_text = font.render(fps, True, pygame.Color("coral"))
    return fps_text


def placePiece(x, y, img):
    gameDisplay.blit(img, (x + width, y + height))


def horizontalCoordinate(x):
    return (x * width) + horizontalOffset


def verticalCoordinate(y):
    return (y * height) + verticalOffset


class TextBox(pygame.sprite.Sprite):
    def __init__(self):
        pygame.sprite.Sprite.__init__(self)
        self.text = ""
        self.font = font
        self.image = self.font.render("", True, [0, 0, 0])
        self.rect = self.image.get_rect()

    def add_chr(self, char):
        global shiftDown
        if char in validChars and not shiftDown:
            self.text += char
        elif char in validChars and shiftDown:
            self.text += shiftChars[validChars.index(char)]
        self.update()

    def update(self, color=[0, 0, 0]):
        old_rect_pos = self.rect
        self.image = self.font.render(self.text, False, color)
        self.rect = self.image.get_rect()
        self.rect = old_rect_pos


class Button():
    def __init__(self, x, y, width, height, color, text):
        self.text = ""
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.color = color
        self.font = font
        self.text = text

    def makeButton(self, window):
        pygame.draw.rect(window, self.color, (self.x, self.y, self.width, self.height), 0)
        if self.text != "":
            text = self.font.render(self.text, True, (0, 0, 0))
            window.blit(text, (self.x + (self.width/2 - text.get_width()/2), self.y + (self.height/2 - text.get_height()/2)))  # centers text

    def isMouseOver(self, position):
        # position = pygame.mouse.set_pos() --> position[0] = x, position[1] = y
        if position[0] > self.x and position[0] < self.x + self.width and position[1] > self.y and position[1] < self.y + self.height:
            return True
        else:
            return False


def make_move():
    i = int(len(board.moveLog)/2 - 20)
    if i < 0:
        i = 0
    heightParameter = i
    create_or_update_board()
    while len(board.moveLog) / 2 >= i:
        thisLine = TextBox()
        thisLine.rect = [display_width * 0.7, (display_height * 0.1) + (25 * (i + 2 - heightParameter)), 200, 200]
        content = str(i + 1)
        if len(board.moveLog) >= 2 * i + 1:
            content += str(board.moveLog[2 * i]) + "---"
            if len(board.moveLog) >= 2 * i + 2:
                content += str(board.moveLog[2 * i + 1])
        thisLine.image = thisLine.font.render(content, True, [0, 0, 0])
        thisLine.text = content
        thisLine.update()
        gameDisplay.blit(thisLine.image, thisLine.rect)
        i += 1


def create_or_update_board():
    print("generating board")
    global board
    gameDisplay.fill(white)
    for i in range(9):
        for j in range(9):
            if i == 0 and j == 0:
                pygame.draw.rect(gameDisplay, white,
                                 ((width * i) + horizontalOffset, (height * j) + verticalOffset, width, height), 0)
                gameDisplay.blit(font.render('test', True, (0, 0, 0)), (width, height+50))
            elif i == 0:
                pygame.draw.rect(gameDisplay, grey,
                                 ((width * i) + horizontalOffset, (height * j) + verticalOffset, width, height), 0)
                test = str(9-j)
                gameDisplay.blit(font.render(test, True, (0, 0, 0)), (width, verticalOffset + height*j + 30))
            elif j == 0:
                pygame.draw.rect(gameDisplay, red,
                                 ((width * i) + horizontalOffset, (height * j) + verticalOffset, width, height), 0)
                gameDisplay.blit(font.render(chr(96+i), True, (0, 0, 0)), (horizontalOffset + 30 + width*i, height+50))
            else:
                if (j + i) % 2 == 0:
                    pygame.draw.rect(gameDisplay, red, ((width * i) + horizontalOffset, (height * j) + verticalOffset, width, height), 0)
                else:
                    pygame.draw.rect(gameDisplay, grey, ((width * i) + horizontalOffset, (height * j) + verticalOffset, width, height), 0)
    # if len(board.moveLog) != 0 and board.moveLog

    # Populate the board
    for i in range(8):
        for j in range(8):
            currentPiece = board.board[i][j]
            if currentPiece == ".":
                pass
            elif currentPiece == "k":
                placePiece(horizontalCoordinate(j), verticalCoordinate(i), blackKingImg)
            elif currentPiece == "p":
                placePiece(horizontalCoordinate(j), verticalCoordinate(i), blackPawnImg)
            elif currentPiece == "q":
                placePiece(horizontalCoordinate(j), verticalCoordinate(i), blackQueenImg)
            elif currentPiece == "b":
                placePiece(horizontalCoordinate(j), verticalCoordinate(i), blackBishopImg)
            elif currentPiece == "n":
                placePiece(horizontalCoordinate(j), verticalCoordinate(i), blackKnightImg)
            elif currentPiece == "r":
                placePiece(horizontalCoordinate(j), verticalCoordinate(i), blackRookImg)
            elif currentPiece == "K":
                placePiece(horizontalCoordinate(j), verticalCoordinate(i), whiteKingImg)
            elif currentPiece == "P":
                placePiece(horizontalCoordinate(j), verticalCoordinate(i), whitePawnImg)
            elif currentPiece == "Q":
                placePiece(horizontalCoordinate(j), verticalCoordinate(i), whiteQueenImg)
            elif currentPiece == "B":
                placePiece(horizontalCoordinate(j), verticalCoordinate(i), whiteBishopImg)
            elif currentPiece == "N":
                placePiece(horizontalCoordinate(j), verticalCoordinate(i), whiteKnightImg)
            elif currentPiece == "R":
                placePiece(horizontalCoordinate(j), verticalCoordinate(i), whiteRookImg)
    print("board done")


if __name__ == '__main__':
    create_or_update_board()
    # init the textbox and set the position and size
    inputBox = TextBox()
    inputBox.rect = [(display_width * 0.7) + 150, display_height * 0.1, 200, 200]
    inputBox.image = inputBox.font.render("", True, [0, 0, 0])
    moveLog = TextBox()
    moveLog.rect = [display_width * 0.7, (display_height * 0.1) + 25, 200, 200]
    moveLog.image = moveLog.font.render("Move Log", True, [0, 0, 0])
    errorBlock = TextBox()
    errorBlock.rect = [(display_width * 0.1), (display_height * 0.85), 200, 200]
    errorBlock.image = errorBlock.font.render("qdzazdzeqfdq", True, [255, 0, 0])
    errorLabel = Button(display_width * 0.1, display_height * 0.8, 75, 50, [255, 255, 255], "Error Log:")  # x, y, width, height, color, text

    enterThePositionBox = TextBox()
    enterThePositionBox.rect = [(display_width * 0.7), display_height * 0.1, 200, 200]
    drawButton = Button(display_width * 0.03, 25, 75, 50, blue, "Draw")  # x, y, width, height, color, text
    resignButton = Button(display_width * 0.1, 25, 75, 50, red, "Resign")
    castleWQButton = Button(display_width * 0.2, 25, 200, 50, red, "White Queen-side castle")
    castleBQButton = Button(display_width * 0.375, 25, 200, 50, red, "Black Queen-side castle")
    castleWKButton = Button(display_width * 0.55, 25, 200, 50, red, "White King-side castle")
    castleBKButton = Button(display_width * 0.725, 25, 200, 50, red, "Black King-side castle")
    # Main loop
    while not gameNotOngoing:
        # get all events
        for e in pygame.event.get():
            position = pygame.mouse.get_pos()
            if e.type == pygame.QUIT:
                gameNotOngoing = True
            if e.type == pygame.QUIT:
                running = False
            if e.type == pygame.KEYUP:
                if e.key in [pygame.K_RSHIFT, pygame.K_LSHIFT]:
                    shiftDown = False
            if e.type == pygame.MOUSEBUTTONDOWN:
                create_or_update_board()
                try:
                    if drawButton.isMouseOver(position) or resignButton.isMouseOver(position):
                        gameNotOngoing = True
                    if castleWKButton.isMouseOver(position):
                        board.castling(True, False)
                        create_or_update_board()
                    if castleWQButton.isMouseOver(position):
                        board.castling(True, True)
                        create_or_update_board()
                    if castleBKButton.isMouseOver(position):
                        board.castling(False, False)
                        create_or_update_board()
                    if castleBQButton.isMouseOver(position):
                        board.castling(False, True)
                        create_or_update_board()
                    print( "yes man, correct castle")
                except Exception as g:
                    print(str(e))
                    errorBlock.text = str(g)
                    errorBlock.update([255, 0, 0])
            if e.type == pygame.KEYDOWN:
                inputBox.add_chr(pygame.key.name(e.key))
                if e.key == pygame.K_SPACE:
                    inputBox.text += " "
                    inputBox.update()
                if e.key in [pygame.K_RSHIFT, pygame.K_LSHIFT]:
                    shiftDown = True
                if e.key == pygame.K_BACKSPACE:
                    inputBox.text = ""
                    inputBox.update()
                    create_or_update_board()
                if e.key == pygame.K_RETURN:
                    create_or_update_board()
                    if len(inputBox.text) > 0:
                        print(inputBox.text)
                        try:
                            coords = board.notationToCords(inputBox.text)
                            board.move(coords[0].row, coords[0].column, coords[1].row, coords[1].column)
                            make_move()
                        except Exception as g:
                            print(type(g))
                            traceback.print_exc()
                            print(str(g))
                            errorBlock.text = str(g)
                            errorBlock.update([255, 0, 0])
                        inputBox.text = ""
                        inputBox.update()
                        print(board.board)
        gameDisplay.blit(inputBox.image, inputBox.rect)
        gameDisplay.blit(moveLog.image, moveLog.rect)
        gameDisplay.blit(errorBlock.image, errorBlock.rect)
        errorLabel.makeButton(gameDisplay)
        gameDisplay.blit(enterThePositionBox.image, enterThePositionBox.rect)
        drawButton.makeButton(gameDisplay)
        resignButton.makeButton(gameDisplay)
        castleBKButton.makeButton(gameDisplay)
        castleBQButton.makeButton(gameDisplay)
        castleWKButton.makeButton(gameDisplay)
        castleWQButton.makeButton(gameDisplay)
        clock.tick(30)

        pygame.display.update()
    pygame.quit()
    quit()
