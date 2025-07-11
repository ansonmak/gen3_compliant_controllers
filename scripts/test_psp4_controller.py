#! /usr/bin/env python
import pygame

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick detected. Make sure the PS4 controller is connected.")

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Detected controller: {joystick.get_name()}")

try:
    while True:
        pygame.event.pump()  # Update joystick events

        # Axes (0–5 typically: LStickX, LStickY, RStickX, RStickY, L2, R2)
        axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        print(f"Axes: {axes}")

        # Buttons (e.g., X, O, Triangle, Square, etc.)
        buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
        print(f"Buttons: {buttons}")

        num_hats = joystick.get_numhats()
        if num_hats > 0:
            hat = joystick.get_hat(0)
            print(f"D-Pad: {hat}")  # (x, y), e.g., (0, 1) for UP

        pygame.time.wait(10)  # Delay (ms)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    pygame.quit()