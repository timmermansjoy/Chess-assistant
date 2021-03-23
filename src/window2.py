import pygame


pygame.init()


display_width = 400
display_height = 400

width = int(display_width / 8)
height = int(display_height / 8)

gameDisplay = pygame.display.set_mode((display_width, display_height), pygame.RESIZABLE)
pygame.display.set_caption('A bit Racey')

black = (0, 0, 0)
white = (255, 255, 255)

clock = pygame.time.Clock()
crashed = False
carImg = pygame.image.load('src/bishop.png')
carImg = pygame.transform.scale(carImg, (int((display_width / 8)), int(display_width / 8)))
font = pygame.font.SysFont("Arial", 18)


def update_fps():
    fps = str(int(clock.get_fps()))
    fps_text = font.render(fps, 1, pygame.Color("coral"))
    return fps_text


def car(x, y):
    gameDisplay.blit(carImg, (x, y))


x = (width * 6)
y = (height * 1)

x2 = (width * 3)
y2 = (height * 4)

while not crashed:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            crashed = True

    gameDisplay.fill(white)

    for i in range(8):
        for j in range(8):
            if (j + i) % 2 == 0:
                pygame.draw.rect(gameDisplay, (0, 0, 0), ((width * i), (height * j), width, height), 0)
            else:
                pygame.draw.rect(gameDisplay, (255, 255, 255), ((width * i), (height * j), width, height), 0)
    car(x, y)
    car(x2, y2)

    gameDisplay.blit(update_fps(), (10, 0))
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            loop = 0
    clock.tick(60)

    pygame.display.update()


pygame.quit()
quit()
