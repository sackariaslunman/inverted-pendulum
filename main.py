import sys, pygame
from time import perf_counter
from lib.cartpoles import CartPoles
from lib.colors import Colors
from numpy import sin, cos, radians

pygame.init()

size = width, height = 1200, 600

screen = pygame.display.set_mode(size)

dt = 0.01
g = 9.81

cart_poles = CartPoles(
    (0.0, 0.5, 0.05, -0.8, 0.8),
    (0.05, 0.05, 0.01, 0.5, 0.05, -24.0, 24.0),
    [
        (radians(10), 0.2, 0.2, 0.005),
        (radians(5), 0.2, 0.15, 0.005),
        (radians(15), 0.2, 0.10, 0.005),
    ],
    g,
    "rk4"
)

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

    state = cart_poles.step(dt, 12*cos(i*dt*2))

    x0 = si_to_pixels(state[0]) + width//2
    y0 = height//2
    pygame.draw.rect(screen, Colors.red, (x0, y0, 20, 10))

    max_x = width//2 + si_to_pixels(cart_poles.max_x)
    min_x = width//2 + si_to_pixels(cart_poles.min_x)
    pygame.draw.rect(screen, Colors.red, (min_x-10, y0, 10, 10))
    pygame.draw.rect(screen, Colors.red, (max_x+20, y0, 10, 10))

    motor_x0 = min_x-100
    motor_sin = si_to_pixels(sin(-state[2])*0.05)
    motor_cos = si_to_pixels(cos(-state[2])*0.05)

    pygame.draw.polygon(screen, Colors.black, [
        (motor_x0+motor_sin, y0+motor_cos),
        (motor_x0+motor_cos, y0-motor_sin),
        (motor_x0-motor_sin, y0-motor_cos),
        (motor_x0-motor_cos, y0+motor_sin),
    ])

    x0 += 10
    for k, ((_,_,l,_), color) in enumerate(zip(cart_poles.poles, [Colors.green, Colors.blue, Colors.purple])):
        x1 = x0 + si_to_pixels(l * sin(state[3+k*2]))
        y1 = y0 + si_to_pixels(-l * cos(state[3+k*2]))
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
        f"Angle: {round(state[2],2)} rad",
    ]
    
    for k, text_k in enumerate(texts):
        text = font.render(text_k, True, Colors.black, Colors.gray)
        text_rect = text.get_rect()
        screen.blit(text,(0,(text_rect.height)*k,text_rect.width,text_rect.height))

    pygame.display.flip()
    i += 1