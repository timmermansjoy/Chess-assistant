import pygame

pygame.init()

validChars = "12345678 abcdefgh"
shiftChars = '12345678 ABCDEFGH'
shiftDown = False

display_width = 1200
display_height = 800

width = int(600 / 8)
height = int(600 / 8)
horizontalOffset = 50
verticalOffset = 100

gameDisplay = pygame.display.set_mode((display_width, display_height), pygame.RESIZABLE)
pygame.display.set_caption('A bit Racey')

red = (100, 25, 25)
white = (255, 255, 255)
grey = (200, 200, 200)

clock = pygame.time.Clock()
Checkmate = False
#importing pieces
whiteBishopImg = pygame.image.load('src/resources/WhiteBishop.svg')
whiteBishopImg = pygame.transform.scale(whiteBishopImg, (width, width))
blackBishopImg = pygame.image.load('src/resources/BlackBishop.svg')
blackBishopImg = pygame.transform.scale(blackBishopImg, (width, width))
whiteKingImg = pygame.image.load('src/resources/WhiteKing.svg')
whiteKingImg = pygame.transform.scale(whiteKingImg, (width, width))
blackKingImg = pygame.image.load('src/resources/BlackKing.svg')
blackKingImg = pygame.transform.scale(blackKingImg, (width, width))
whiteQueenImg = pygame.image.load('src/resources/WhiteQueen.svg')
whiteQueenImg = pygame.transform.scale(whiteQueenImg, (width, width))
blackQueenImg = pygame.image.load('src/resources/BlackQueen.svg')
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
                textBox.text = textBox.text[:-1]
                textBox.update()
            if e.key == pygame.K_RETURN:
                if len(textBox.text) > 0:
                    print(textBox.text)
                    textBox.text = ""
                    textBox.update()

    gameDisplay.fill(white)

    for i in range(8):
        for j in range(8):
            if (j + i) % 2 == 0:
                pygame.draw.rect(gameDisplay, red,
                                 ((width * i) + horizontalOffset, (height * j) + verticalOffset, width, height), 0)
            else:
                pygame.draw.rect(gameDisplay, grey,
                                 ((width * i) + horizontalOffset, (height * j) + verticalOffset, width, height), 0)

    bishop(x, y)
    bishop(x2, y2)
    bishop(x3, y3)
    placePiece(verticalCoordinate(verticalCoordinate(4)), horizontalCoordinate(0), blackKingImg)

    gameDisplay.blit(update_fps(), (10, 0))
    gameDisplay.blit(textBox.image, textBox.rect)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            loop = 0
    clock.tick(60)

    pygame.display.update()

pygame.quit()
quit()
