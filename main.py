import sys, pygame
from time import perf_counter
from math import sin, cos, radians
from lib.cartpoles import Cart, Pole, DCMotor
from lib.colors import Colors

pygame.init()

size = width, height = 1200, 600

screen = pygame.display.set_mode(size)

dt = 0.001
g = 9.81

cart = Cart(0.5, 0.05, 0, -0.8, 0.8, Colors.red,
    DCMotor(-12, 12, 0.05, 0.5, 0.05, 0.01, 0.05, Colors.black),
    Pole(0.2, radians(10), 0.2, 0.005, Colors.green, 
        Pole(0.2, radians(5), 0.15, 0.005, Colors.blue, 
            Pole(0.2, radians(15), 0.10, 0.005, Colors.purple, None
            )
        )
    )
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

    cart.update(dt, 12*cos(i*dt*2), g)

    x0 = si_to_pixels(cart.position()) + width//2
    y0 = height//2
    pygame.draw.rect(screen, cart.color, (x0, y0, 20, 10))

    max_x = width//2 + si_to_pixels(cart.max_x)
    min_x = width//2 + si_to_pixels(cart.min_x)
    pygame.draw.rect(screen, cart.color, (min_x-10, y0, 10, 10))
    pygame.draw.rect(screen, cart.color, (max_x+20, y0, 10, 10))

    motor_x0 = min_x-100
    motor_sin = si_to_pixels(sin(-cart.motor.angle())*0.05)
    motor_cos = si_to_pixels(cos(-cart.motor.angle())*0.05)

    pygame.draw.polygon(screen, cart.motor.color, [
        (motor_x0+motor_sin, y0+motor_cos),
        (motor_x0+motor_cos, y0-motor_sin),
        (motor_x0-motor_sin, y0-motor_cos),
        (motor_x0-motor_cos, y0+motor_sin),
    ])

    x0 += 10
    for pole in cart:
        x1 = x0 + si_to_pixels(pole.l * sin(pole.angle()))
        y1 = y0 + si_to_pixels(-pole.l * cos(pole.angle()))
        pygame.draw.line(screen, pole.color, (x0, y0), (x1, y1), 10)
        x0 = x1
        y0 = y1
    
    texts = [
        f"Time: {round(i*dt,2)} s",
        f"",
        f"Cart:",
        f"Position: {round(cart.position(),2)} m",
        f"Velocity: {round(cart.velocity(),2)} m/s",
        f"Acceleration: {round(cart.acceleration(),2)} m/s^2",
        f"",
        f"Motor:",
        f"Angle: {round(cart.motor.angle(),2)} rad",
        f"Angular velocity: {round(cart.motor.angular_velocity(),2)} rad/s",
    ]
    
    for k, text_k in enumerate(texts):
        text = font.render(text_k, True, Colors.black, Colors.gray)
        text_rect = text.get_rect()
        screen.blit(text,(0,(text_rect.height)*k,text_rect.width,text_rect.height))

    pygame.display.flip()
    i += 1