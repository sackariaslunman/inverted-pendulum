import sys, pygame
from time import time
from math import pi, sin ,cos
from cartpole import Cart, Pole, DCMotor
pygame.init()

size = width, height = 1200, 600

screen = pygame.display.set_mode(size)

dt = 0.0005
g = 9.81

system = DCMotor(12, -12, 0.05, 0, 0.5, 0.5, 0.05, 0.01, (0,0,0), 
    Cart(0.1, 0.05, 0, -0.8, 0.8, (255,0,0), 
        Pole(0.1, 10/180*pi, 0.2, 0.01, (0,255,0), 
            Pole(0.1, 5/180*pi, 0.2, 0.01, (0,0,255), 
                Pole(0.1, 15/180*pi, 0.2, 0.01, (255,0,255), None)
            )
        )
    )
)

def si_to_pixels(x: float):
    return int(x * 500)

last_update = 0
start_time = time()
i = 0

while True:
    current_time = time()
    if current_time > dt + last_update:
        last_update = current_time
    else:
        continue

    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()

    system.update(dt, 5*sin(0.001*i) + 2*cos(0.005*i), g)

    screen.fill((255,255,255))

    x0 = si_to_pixels(system.cart.x()) + width//2
    y0 = height//2
    pygame.draw.rect(screen, system.cart.color, (x0, y0, 20, 10))

    max_x = width//2 + si_to_pixels(system.cart.max_x)
    min_x = width//2 + si_to_pixels(system.cart.min_x)
    pygame.draw.rect(screen, system.cart.color, (min_x-10, y0, 10, 10))
    pygame.draw.rect(screen, system.cart.color, (max_x+20, y0, 10, 10))

    motor_x0 = min_x-100
    motor_x1 = si_to_pixels(sin(-system.angle())*0.05)
    motor_y1 = si_to_pixels(cos(-system.angle())*0.05)

    pygame.draw.polygon(screen, system.color, [
        (motor_x0+motor_x1, y0+motor_y1),
        (motor_x0+motor_y1, y0-motor_x1),
        (motor_x0-motor_x1, y0-motor_y1),
        (motor_x0-motor_y1, y0+motor_x1),
    ])

    x0 += 10
    for pole in iter(system.cart):
        x1 = x0 + si_to_pixels(pole.l * sin(pole.angle()))
        y1 = y0 + si_to_pixels(-pole.l * cos(pole.angle()))
        pygame.draw.line(screen, pole.color, (x0, y0), (x1, y1), 10)
        x0 = x1
        y0 = y1
    
    pygame.display.flip()
    i += 1