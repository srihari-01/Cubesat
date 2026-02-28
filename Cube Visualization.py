import serial
import re
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

# --- CONFIGURATION ---
SERIAL_PORT = "COM10" 
BAUD_RATE = 115200

# Initialize Serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    print(f"--- Connected to {SERIAL_PORT} at {BAUD_RATE} baud ---")
except Exception as e:
    print(f"FATAL ERROR: Could not open {SERIAL_PORT}. Close Arduino Serial Monitor first!")
    exit()

def draw_axes():
    """ Draws X, Y, Z axes for orientation reference """
    glLineWidth(3)
    glBegin(GL_LINES)
    
    # X-axis (Red) - Now controlled by Pitch data
    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(0, 0, 0)
    glVertex3f(3, 0, 0)
    
    # Y-axis (Green) - Now controlled by Roll data
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 3, 0)
    
    # Z-axis (Blue)
    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, 3)
    
    glEnd()
    glLineWidth(1)

def draw_colored_cube():
    """ Defines a solid cube with different colored faces """
    vertices = (
        (1, -1, -1), (1, 1, -1), (-1, 1, -1), (-1, -1, -1),
        (1, -1, 1), (1, 1, 1), (-1, -1, 1), (-1, 1, 1)
    )
    surfaces = (
        (0,1,2,3), (3,2,7,6), (6,7,5,4), (4,5,1,0), (1,5,7,2), (4,0,3,6)
    )
    colors = (
        (1,0,0), (0,1,0), (0,0,1), (1,1,0), (1,0,1), (0,1,1)
    )

    glBegin(GL_QUADS)
    for i, surface in enumerate(surfaces):
        r, g, b = colors[i]
        glColor3f(r * 0.6, g * 0.6, b * 0.6) # Slightly dimmed for contrast
        for vertex in surface:
            glVertex3fv(vertices[vertex])
    glEnd()

def draw_text(position, text, font):
    """ Draws 2D text overlay """
    text_surface = font.render(text, True, (255, 255, 255))
    text_data = pygame.image.tostring(text_surface, "RGBA", True)
    glWindowPos2d(position[0], position[1])
    glDrawPixels(text_surface.get_width(), text_surface.get_height(), 
                 GL_RGBA, GL_UNSIGNED_BYTE, text_data)

def main():
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    pygame.display.set_caption("LoRa IMU - Swapped Axes")
    
    glEnable(GL_DEPTH_TEST)
    font = pygame.font.SysFont('monospace', 20)
    
    roll = 0.0
    pitch = 0.0
    clock = pygame.time.Clock()

    print("Listening for LoRa packets... Press Ctrl+C in terminal to stop.")

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                ser.close()
                pygame.quit()
                quit()

        # --- DATA PROCESSING & PRINTING ---
        while ser.in_waiting > 0:
            try:
                # Read line from serial
                raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if raw_line:
                    # PRINTING TO TERMINAL
                    print(f"RX: {raw_line}") 

                    # Parse the data: Looks for "IMU:float,float"
                    match = re.search(r"IMU:([-+]?\d*\.\d+|[-+]?\d+),([-+]?\d*\.\d+|[-+]?\d+)", raw_line)
                    if match:
                        roll = float(match.group(1))
                        pitch = float(match.group(2))
            except Exception as e:
                print(f"Serial Error: {e}")

        # --- RENDERING ---
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
        glTranslatef(0.0, 0.0, -7)
        
        # --- SWAPPED AXES ---
        # Original: glRotatef(roll, 1, 0, 0) | glRotatef(pitch, 0, 1, 0)
        # New: Roll rotates Y, Pitch rotates X
        glRotatef(roll, 0, 1, 0)   # Now Roll affects the Y-axis
        glRotatef(pitch, 1, 0, 0)  # Now Pitch affects the X-axis
        
        draw_axes()
        draw_colored_cube()

        # UI Overlay
        draw_text((20, 560), f"ROLL (Y):  {roll:>7.2f}", font)
        draw_text((20, 530), f"PITCH (X): {pitch:>7.2f}", font)
        draw_text((20, 30), "X: Red | Y: Green | Z: Blue", font)

        pygame.display.flip()
        clock.tick(60) 

if __name__ == "__main__":
    main()