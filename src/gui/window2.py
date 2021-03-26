import pygame

from board import Board


pygame.init()
board = Board()
validChars = "12345678 abcdefgh"
shiftChars = '12345678 ABCDEFGH'
shiftDown = False

display_width = 1200
display_height = 800

width = int(600 / 8)
height = int(600 / 8)
horizontalOffset = 50
verticalOffset = 100

gameDisplay = pygame.display.set_mode((display_width, display_height))
pygame.display.set_caption('A bit Racey')

red = (100, 25, 25)
white = (255, 255, 255)
grey = (200, 200, 200)

clock = pygame.time.Clock()
Checkmate = False
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

font = pygame.font.SysFont("Arial", 18)


def update_fps():
    fps = str(int(clock.get_fps()))
    fps_text = font.render(fps, 1, pygame.Color("coral"))
    return fps_text


def bishop(x, y):
    placePiece(x, y, whiteBishopImg)


def placePiece(x, y, img):
    gameDisplay.blit(img, (x, y))


def horizontalCoordinate(x):
    return (x * width) + horizontalOffset


def verticalCoordinate(x):
    return (x * height) + verticalOffset


x = (width * 6) + horizontalOffset
y = (height * 1) + verticalOffset

x2 = (width * 3) + horizontalOffset
y2 = (height * 3) + verticalOffset

x3 = (width * 0) + horizontalOffset
y3 = (height * 3) + verticalOffset


class TextBox(pygame.sprite.Sprite):
    def __init__(self):
        pygame.sprite.Sprite.__init__(self)
        self.text = ""
        self.font = font
        self.image = self.font.render("Enter the position", False, [0, 0, 0])
        self.rect = self.image.get_rect()

    def add_chr(self, char):
        global shiftDown
        if char in validChars and not shiftDown:
            self.text += char
        elif char in validChars and shiftDown:
            self.text += shiftChars[validChars.index(char)]
        self.update()

    def update(self):
        old_rect_pos = self.rect
        self.image = self.font.render(self.text, False, [0, 0, 0])
        self.rect = self.image.get_rect()
        self.rect = old_rect_pos


# init the textbox and set the position and size
textBox = TextBox()
textBox.rect = [display_width * 0.7, display_height * 0.5, 200, 200]


def create_or_update_board():
    print("generating board")
    global board
    gameDisplay.fill(white)
    for i in range(8):
        for j in range(8):
            if (j + i) % 2 == 0:
                pygame.draw.rect(gameDisplay, red,
                                 ((width * i) + horizontalOffset, (height * j) + verticalOffset, width, height), 0)
            else:
                pygame.draw.rect(gameDisplay, grey,
                                 ((width * i) + horizontalOffset, (height * j) + verticalOffset, width, height), 0)
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
    # Main loop
    while not Checkmate:
        # get all events
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                Checkmate = True
            if e.type == pygame.QUIT:
                running = False
            if e.type == pygame.KEYUP:
                if e.key in [pygame.K_RSHIFT, pygame.K_LSHIFT]:
                    shiftDown = False
            if e.type == pygame.KEYDOWN:
                textBox.add_chr(pygame.key.name(e.key))
                if e.key == pygame.K_SPACE:
                    textBox.text += " "
                    textBox.update()
                if e.key in [pygame.K_RSHIFT, pygame.K_LSHIFT]:
                    shiftDown = True
                if e.key == pygame.K_BACKSPACE:
                    textBox.text = ""
                    textBox.update()
                    create_or_update_board()
                if e.key == pygame.K_RETURN:
                    create_or_update_board()
                    if len(textBox.text) > 0:
                        print(textBox.text)
                        coords = board.notationToCords(textBox.text)
                        textBox.text = ""
                        textBox.update()
                        board.move(coords[0].row, coords[0].column, coords[1].row, coords[1].column)
                        print(board.board)
                        create_or_update_board()

        gameDisplay.blit(update_fps(), (10, 0))
        gameDisplay.blit(textBox.image, textBox.rect)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                loop = 0
        clock.tick(60)

        pygame.display.update()

    pygame.quit()
    quit()
