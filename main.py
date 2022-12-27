import sys, pygame
from time import time
from math import pi, sin ,cos
from cartpole import Cart, Pole
pygame.init()

size = width, height = 1200, 600

screen = pygame.display.set_mode(size)

dt = 0.0005
runtime = 10
N = int(runtime / dt)
g = 9.81

cart = Cart(0.1, 0.05, 0, -0.8, 0.8, (255,0,0), 
    Pole(0.1, 10/180*pi, 0.2, 0.01, (0,255,0), 
        Pole(0.1, 5/180*pi, 0.2, 0.01, (0,0,255), 
            Pole(0.1, 15/180*pi, 0.2, 0.01, (255,0,255), None)
        )
    )
)

def si_to_pixels(x: float):
    return int(x * 500)

last_update = 0
start_time = time()

while True:
    current_time = time()
    if current_time > dt + last_update:
        last_update = current_time
    else:
        continue

    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()

    for i in range(1,N):
        cart.update(dt, 0, g)

        x0 = si_to_pixels(cart.x()) + width//2
        y0 = height//2

        screen.fill((255,255,255))
        x0 = si_to_pixels(cart.x()) + width//2
        y0 = height//2
        pygame.draw.rect(screen, cart.color, (x0, y0, 20, 10))

        max_x = width//2 + si_to_pixels(cart.max_x)
        min_x = width//2 + si_to_pixels(cart.min_x)

        pygame.draw.rect(screen, cart.color, (min_x-10, y0, 10, 10))
        pygame.draw.rect(screen, cart.color, (max_x+20, y0, 10, 10))

        x0 += 10
        for pole in iter(cart):
            x1 = x0 + si_to_pixels(pole.l * sin(pole.angle()))
            y1 = y0 + si_to_pixels(-pole.l * cos(pole.angle()))
            pygame.draw.line(screen, pole.color, (x0, y0), (x1, y1), 10)
            x0 = x1
            y0 = y1
        
        pygame.display.flip()