import pygame
import sys;
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import numpy as np;
import cv2;

def main():
    pygame.init()
    pygame.display.set_mode((600,600), DOUBLEBUF|OPENGL)
main()
def cvimage_to_pygame(image):
    return pygame.image.frombuffer(image.tostring(),image.shape[1::-1],"RGB");
    
img = cv2.cvtColor(cv2.imread('jeffery-pyOpenGl-testImage.jpg'),cv2.COLOR_BGR2RGB);
textureData = pygame.image.tostring(cvimage_to_pygame(img), "RGB", 1)
width = img.shape[1]
height = img.shape[0]

im = glGenTextures(1)
glBindTexture(GL_TEXTURE_2D, im)

glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, textureData)
glEnable(GL_TEXTURE_2D)

def wall():
    vector1 = [0,5,0];
    vector2 = [1,0,1];
    center = [0,0,-15];
    vector3 = np.cross(vector1,vector2);
    vector1 = vector1 / np.linalg.norm(vector1);
    vector2 = vector2 / np.linalg.norm(vector2);
    vector3 = vector3 / np.linalg.norm(vector3);
    glBegin(GL_POLYGON)
    glTexCoord2f(0,0)
    point = center + -1*width*vector3 -1*height*vector1
    glVertex3f(point[0],point[1],point[2]);
    glTexCoord2f(0,1)
    point = center + -1*width*vector3 +1*height*vector1
    glVertex3f(point[0],point[1],point[2]);
    glTexCoord2f(1,1)
    point = center +1*width*vector3 +1*height*vector1
    glVertex3f(point[0],point[1],point[2]);
    glTexCoord2f(1,0)
    point = center + 1*width*vector3 -1*height*vector1
    glVertex3f(point[0],point[1],point[2]);
    glEnd()
    '''glEnable(GL_POINT_SMOOTH);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointParameteri(GL_POINT_SIZE_MIN,0);
    glPointParameteri(GL_POINT_SIZE_MAX,100);
    glPointParameteri(GL_POINT_FADE_THRESHOLD_SIZE,100);
    glPointParameterfv(GL_POINT_DISTANCE_ATTENUATION,[1,0.01,0]);
    glPointSize(100);
    glBegin(GL_POINTS);
    glVertex3f(10,0,4500)
    glVertex3f(-10,0,4500)
    glVertex3f(10,10,4500)
    glVertex3f(-10,10,4500)
    glEnd();
    glDisable(GL_POINT_SMOOTH);
    glDisable(GL_PROGRAM_POINT_SIZE)'''
angle = 1;
direction = 0.5;
index = 0;
vec = [[1,0,0],[0,1,0],[0,0,1]];
verticies = (
    (width,-1*height,-16),
    (width,height,-16),
    (-1*width,height,-16),
    (-1*width,-1*height,-16),
    (width,-1*height,500),
    (width,height,500),
    (-1*width,-1*height,500),
    (-1*width,height,500)
    )

edges = (
    (0,1),
    (0,3),
    (0,4),
    (2,1),
    (2,3),
    (2,7),
    (6,3),
    (6,4),
    (6,7),
    (5,1),
    (5,4),
    (5,7)
    )


def Cube():
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(verticies[vertex])
    glEnd()
breakflag = 0;
glutInit(sys.argv);
glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
while True:
    if angle >360:
        angle = 0;
    if angle == 0:
        if index <=1:
            index  = index +1;
        else:
            index = 0;
    angle = angle + direction;
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit();
            breakflag = 1;
    if breakflag == 1:
        break;
    glLoadIdentity()
    gluPerspective(45, 1, 0.05, 10000)
    glTranslatef(0,0,-5000)
    glRotatef(angle,vec[index][0],vec[index][1],vec[index][2])
    
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    wall()
    glLoadIdentity()
    glTranslatef(0,0,-2000)
    w,v = np.linalg.eig([[1,0,0],[0,0,1],[0,0,1]]);
    glutSolidSphere(100,1000,1000);
    #Cube()
    pygame.display.flip()
    pygame.time.wait(10)
