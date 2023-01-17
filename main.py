import sys, pygame
from time import perf_counter
from lib.cartpoles import CartPoleSystem
from lib.colors import Colors
from numpy import sin, cos, radians
import numpy as np

pygame.init()

size = width, height = 1200, 600

screen = pygame.display.set_mode(size)

dt = 0.01
g = 9.81

system1 = CartPoleSystem(
    (0.0, 0.5, 0.05, -0.8, 0.8, Colors.red),
    (0.05, 0.05, 0.01, 0.5, 0.05, -24.0, 24.0, Colors.black),
    [
        (radians(10), 0.2, 0.2, 0.005, Colors.green),
        # (radians(5), 0.3, 0.15, 0.005, Colors.blue),
        # (radians(15), 0.4, 0.1, 0.005, Colors.purple),
    ],
    g,
    "rk4",
    "nonlinear"
)

systems = [system1]

def si_to_pixels(x: float):
    return int(x * 500)

last_update = 0
start_time = perf_counter()
i = 0

font = pygame.font.Font('freesansbold.ttf', 20)

while True:
    current_time = perf_counter()-start_time
    if current_time > dt + last_update:
        last_update = current_time
    else:
        continue

    for event in pygame.event.get():
        if event.type == pygame.QUIT: 
            sys.exit()

    screen.fill(Colors.gray)

    for system in systems:

        state, _ = system.step(dt, np.array([[14*cos(2*dt*i)]]))
        state = state.T[0]

        x = state[0]

        x0 = si_to_pixels(x) + width//2
        y0 = height//2
        pygame.draw.rect(screen, system.cart_color, (x0, y0, 20, 10))

        max_x = width//2 + si_to_pixels(system.max_x)
        min_x = width//2 + si_to_pixels(system.min_x)
        pygame.draw.rect(screen, system.cart_color, (min_x-10, y0, 10, 10))
        pygame.draw.rect(screen, system.cart_color, (max_x+20, y0, 10, 10))

        theta_m = x/system.r
        motor_x0 = min_x-100
        motor_sin = si_to_pixels(sin(-theta_m)*0.05)
        motor_cos = si_to_pixels(cos(-theta_m)*0.05)

        pygame.draw.polygon(screen, system.motor_color, [
            (motor_x0+motor_sin, y0+motor_cos),
            (motor_x0+motor_cos, y0-motor_sin),
            (motor_x0-motor_sin, y0-motor_cos),
            (motor_x0-motor_cos, y0+motor_sin),
        ])

        x0 += 10
        for k, ((_,_,l,_),color) in enumerate(zip(system.poles,system.pole_colors)):
            x1 = x0 + si_to_pixels(l * sin(state[2+k*2]))
            y1 = y0 + si_to_pixels(-l * cos(state[2+k*2]))
            pygame.draw.line(screen, color, (x0, y0), (x1, y1), 10)
            x0 = x1
            y0 = y1
        
        texts = [
            f"Time: {round(i*dt,2)} s",
            f"",
            f"Cart:",
            f"Position: {round(state[0],2)} m",
            f"Velocity: {round(state[1],2)} m/s",
            f"",
            f"Motor:",
            f"Angle: {round(theta_m,2)} rad",
        ]
        
        for k, text_k in enumerate(texts):
            text = font.render(text_k, True, Colors.black, Colors.gray)
            text_rect = text.get_rect()
            screen.blit(text,(0,(text_rect.height)*k,text_rect.width,text_rect.height))

    i += 1
    pygame.display.flip()
    