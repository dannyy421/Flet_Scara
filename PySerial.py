import pygame
import serial, time

pygame.init()
joystick_count = pygame.joystick.get_count()
if joystick_count > 0:
    my_joystick = pygame.joystick.Joystick(0) 
    my_joystick.init()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                button_number = event.button
                print(f"Button {button_number} pressed")

            if event.type == pygame.JOYBUTTONUP:
                button_number = event.button
                print(f"Button {button_number} released")

else:
    print("No joysticks connected.")

