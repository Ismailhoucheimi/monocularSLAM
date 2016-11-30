# -*- coding: utf-8 -*-
"""
Created on Fri Apr  1 18:03:59 2016

@author: jeffery
"""
import pygame;
import sys;
from pygame.locals import *;global PGscreen;
from OpenGL.GL import *;
from OpenGL.GLU import *;
from OpenGL.GLUT import *;
import sharedTypes as ST;
import time;
import cv2;
import numpy as np;
#import FTGL;
global PGscreen;
global imageID;
global observerLoc;
global imageID;
imageID = 0;
global screenShotFlag;
screenShotFlag = False;
observerLoc = [0, 0,20];
global rotation;
rotation = [0,0];
def plotAxis():
    glColor4f(0,1,1,0.4);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(5,0,0);
    glEnd();
    glColor4f(1,0,1,0.4);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(0,5,0);
    glEnd();
    glColor4f(1,1,0,0.4);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(0,0,5);
    glEnd();
def plotTrajectory(X):
    glColor4f(1,1,1,0.4);
    glBegin(GL_LINE_STRIP);
    for i in range(0,X.shape[0]):
        glVertex3f(X[i,0],X[i,1],X[i,2]);
    glEnd();
def plotEllipsoid(x,Cov):
    global observerLoc;
    global rotation;
    glEnable(GL_BLEND);
    glColor4f(1,1,1,0.3)
    glBlendFunc(GL_SRC_ALPHA , GL_ONE_MINUS_SRC_ALPHA);
    w,v=np.linalg.eig(Cov);
    glTranslatef(x[0],x[1],x[2]);
    angle = np.arccos(np.dot(v[:,0],[1,0,0])/np.linalg.norm(v[:,0]));
    glRotatef(angle,v[0,0],v[1,0],v[2,0]);
    glScalef(np.sqrt(w[0]),np.sqrt(w[1]),np.sqrt(w[2]));
    glutSolidSphere(10,1000,1000);
    glLoadIdentity();
    glRotatef(180,1,0,0);
    gluPerspective(100, 1, 0.05, 10**10);
    glTranslatef(observerLoc[0],observerLoc[1],observerLoc[2]);
    glRotatef(rotation[0],0,1,0);
    glRotatef(rotation[1],1,0,0);
    glDisable(GL_BLEND);
    glColor4f(1,1,1,1);
def plotEllipsoidMask(x,Cov):
    global observerLoc;
    global rotation;
    glLoadIdentity();
    gluPerspective(100, 1, 0.05, 10**10);
    glRotatef(180,1,0,0);
    glTranslatef(observerLoc[0],observerLoc[1],observerLoc[2]);
    glRotatef(rotation[0],0,1,0);
    glRotatef(rotation[1],1,0,0);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glEnable(GL_BLEND);
    glColor4f(1,1,1,1)
    glBlendFunc(GL_SRC_ALPHA , GL_ONE_MINUS_SRC_ALPHA);
    w,v=np.linalg.eig(Cov);
    glTranslatef(x[0],x[1],x[2]);
    angle = np.arccos(np.dot(v[:,0],[1,0,0])/np.linalg.norm(v[:,0]));
    glRotatef(angle,v[0,0],v[1,0],v[2,0]);
    glScalef(np.sqrt(w[0]),np.sqrt(w[1]),np.sqrt(w[2]));
    glutSolidSphere(10,1000,1000);
    glDisable(GL_BLEND);
    glFinish();
    glPixelStorei(GL_PACK_ALIGNMENT, 4)
    glPixelStorei(GL_PACK_ROW_LENGTH, 0)
    glPixelStorei(GL_PACK_SKIP_ROWS, 0)
    glPixelStorei(GL_PACK_SKIP_PIXELS, 0)    
    data = glReadPixels(0, 0, 600, 600, GL_RGB,  GL_UNSIGNED_INT)  
    #cv2.cvtColor(temp, tempImage, CV_BGR2RGB);
    #cv2.flip(tempImage, temp, 0);
    glLoadIdentity();
    glRotatef(180,1,0,0);
    gluPerspective(100, 1, 0.05, 10**10);
    glTranslatef(observerLoc[0],observerLoc[1],observerLoc[2]);
    glRotatef(rotation[0],0,1,0);
    glRotatef(rotation[1],1,0,0);
    img = np.asarray(data[:,:,:],dtype =uint8);
    img = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY);
    img[img>0.5] = 1;
    img[img<=0.5] = 0;
    return img;
def plotCamera(x,vec):
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA , GL_ONE_MINUS_SRC_ALPHA);
    global observerLoc;
    global rotation;
    glColor4f(1,0,1,0.3)
    glTranslatef(x[0],x[1],x[2]);
    vector = np.cross([0,0,1],vec);
    angle = np.arccos(np.dot(vec,[0,0,1])/np.linalg.norm(vec));
    glRotatef(angle,vector[0],vector[1],vector[2]);
    glutSolidCone(20,100,1000,1000);
    glLoadIdentity();
    gluPerspective(100, 1, 0.05, 10**10);
    glRotatef(180,1,0,0);
    glTranslatef(observerLoc[0],observerLoc[1],observerLoc[2]);
    glRotatef(rotation[0],0,1,0);
    glRotatef(rotation[1],1,0,0);
    glDisable(GL_BLEND);
    glColor4f(1,1,1,1);
def plotlandmarks(vec1,vec2,center,img):
    glColor4f(1,1,1,1);
    img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB);
    textureData = np.flipud(img).tostring();#pygame.image.tostring(pygame.image.frombuffer(img.tostring(),img.shape[1::-1],"RGB"), "RGB", 1);
    width = img.shape[1];
    height = img.shape[0];
    im = glGenTextures(1);
    glBindTexture(GL_TEXTURE_2D, im);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, textureData);
    vec3 = np.cross(vec1,vec2);
    vec1 = vec1 / np.linalg.norm(vec1);
    vec2 = vec2 / np.linalg.norm(vec2);
    vec3 = vec3 / np.linalg.norm(vec3);
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_POLYGON);
    glTexCoord2f(0,0);
    point = center + -1*width*vec3 -1*height*vec1;
    glVertex3f(point[0],point[1],point[2]);
    glTexCoord2f(0,1);
    point = center + -1*width*vec3 +1*height*vec1;   
    glVertex3f(point[0],point[1],point[2]);
    glTexCoord2f(1,1);
    point = center +1*width*vec3 +1*height*vec1;
    glVertex3f(point[0],point[1],point[2]);
    glTexCoord2f(1,0);
    point = center + 1*width*vec3 -1*height*vec1;   
    glVertex3f(point[0],point[1],point[2]);
    glEnd();rotation
    glDisable(GL_TEXTURE_2D);
    glColor4f(1,1,1,1);
def plotParticles(X):
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA , GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(1,1,1,0.6)
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointParameteri(GL_POINT_SIZE_MIN,0);
    glPointParameteri(GL_POINT_SIZE_MAX,200);
    glPointParameteri(GL_POINT_FADE_THRESHOLD_SIZE,200);
    glPointParameterfv(GL_POINT_DISTANCE_ATTENUATION,[1,0.1,0]);
    glPointSize(200);
    glBegin(GL_POINTS);
    glVertex3f(X[0],X[1],X[2]);
    glEnd();
    glDisable(GL_POINT_SMOOTH);
    glDisable(GL_BLEND);
    glColor4f(1,1,1,1);
def plotFlip():
    global observerLoc;
    global rotation;
    global imageID;
    global screenShotFlag;
    glFinish();
    if screenShotFlag:
        imageID += 1;
        glPixelStorei(GL_PACK_ALIGNMENT, 4)
        glPixelStorei(GL_PACK_ROW_LENGTH, 0)
        glPixelStorei(GL_PACK_SKIP_ROWS, 0)
        glPixelStorei(GL_PACK_SKIP_PIXELS, 0)
        data = glReadPixels(0, 0, 600, 600, GL_RGBA, GL_UNSIGNED_BYTE)
        surface = pygame.image.fromstring(data, (600, 600), 'RGBA', 1)
        filename = 'ScreenShot6-'+str(imageID)+'.jpg';
        pygame.image.save(surface, filename);
    pygame.display.flip();
    glLoadIdentity();
    gluPerspective(100, 1, 0.05, 10**10);
    glRotatef(180,1,0,0);
    glTranslatef(observerLoc[0],observerLoc[1],observerLoc[2]);
    glRotatef(rotation[0],0,1,0);
    glRotatef(rotation[1],1,0,0);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glColor4f(1,1,1,1);
def plotInit():
    global observerLoc;
    global rotation;
    global PGscreen;
    pygame.init();
    glutInit(sys.argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    PGscreen = pygame.display.set_mode((600,600), DOUBLEBUF|OPENGL);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glLoadIdentity();
    gluPerspective(100, 1, 0.05, 10**10);
    glRotatef(180,1,0,0);
    glTranslatef(observerLoc[0],observerLoc[1],observerLoc[2]);
    glRotatef(rotation[0],0,1,0);
    glRotatef(rotation[1],1,0,0);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glColor4f(1,1,1,1);
def plotClose():
    pygame.quit();
if __name__ == "__main__":
    global observerLoc;
    global rotation;
    plotInit();
    img = cv2.imread('./jeffery-pyOpenGl-testImage.jpg');
    img2 = cv2.imread('./calibration/chole.jpg');
    #img = cv2.resize(img,(int(0.25*img.shape[0]),int(0.25*img.shape[1])))
    '''for i in range(0,100):i
        print(i);        
        for j in range(0,i): 
            glTranslatef(0,0,-1);
            plotParticles([0,0,0]);
        plotFlip();
        time.sleep(0.5);'''
    breakflag = 0;
    mouseleft = 0;
    mouseright = 0;
    prelocX = 0;
    prelocY = 0;
    #font = FTGL.PolygonFont('/usr/share/fonts/truetype/ubuntu-font-family/Ubuntu-C.ttf'); 
    #font.FaceSize(80); 
    while True:
        #plotAxis();
        plotlandmarks([0,-5,0],[1,1,-1],[-1000,0,2000],img);
        plotlandmarks([1,-5,0],[-1,1,-1],[800,0,1000],img2);
        #plotCamera([0,0,200],[0,0,1]);
        #arr = plotEllipsoidMask([0,0,1000],np.diag((1000,10,1000)));
        #plotParticles([-4.13008103,  9.08617826 , 0.61951215]);
        #plotParticles([ 3.26518033 , 8.90956217, -3.15567733]);
        #plotParticles([-0.80494419 , 5.09174002, -8.56891174]);
        #plotParticles([  -7.01837502,    5.31028857 , 345.25202702]);   
        '''glTranslatef(-200,200,-500);
        glColor4f(1,1,1,1);
        font.Render('1');
        glLoadIdentity();
        gluPerspective(100, 1, 0.05, 10000);
        glTranslatef(observerLoc[0],observerLoc[1],observerLoc[2]);
        glRotatef(rotation[0],0,1,0);
        glRotatef(rotation[1],1,0,0600  );
        glTranslatef(200,200,-500);
        font.Render('2');
        glLoadIdentity();
        gluPerspective(100, 1, 0.05, 10000);
        glTranslatef(observerLoc[0],observerLoc[1],observerLoc[2]);
        glRotatef(rotation[0],0,1,0);
        glRotatef(rotation[1],1,0,0);
        glTranslatef(200,-200,-500);
        font.Render('3');gluPerspective
        glLoadIdentity();
        gluPerspective(100, 1, 0.05, 10000);
        glTranslatef(observerLoc[0],observerLoc[1],observerLoc[2]);
        glRotatef(rotation[0],0,1,0);
        glRotatef(rotation[1],1,0,0);
        glTranslatef(-200,-200,-500);
        font.Render('4');
        glLoadIdentity();
        gluPerspective(100, 1, 0.05, 10000);
        glTranslatef(observerLoc[0],observerLoc[1],observerLoc[2]);
        glRotatef(rotation[0],0,1,0);
        glRotatef(rotation[1],1,0,0);
        glTranslatef(-200,200,-1000);
        font.Render('5');
        glLoadIdentity();
        gluPerspective(100, 1, 0.05, 10000);
        glTranslatef(observerLoc[0],observerLoc[1],observerLoc[2]);
        glRotatef(rotation[0],0,1,0);
        glRotatef(rotation[1],1,0,0);
        glTranslatef(200,200,-1000);
        font.Render('6');
        glLoadIdentity();
        gluPerspective(100, 1, 0.05, 10000);
        glTranslatef(observerLoc[0],observerLoc[1],observerLoc[2]);
        glRotatef(rotation[0],0,1,0);
        glRotatef(rotation[1],1,0,0);
        glTranslatef(200,-200,-1000);
        font.Render('7');
        glLoadIdentity();
        gluPerspective(100, 1, 0.05, 10000);
        glTranslatef(observerLoc[0],observerLoc[1],observerLoc[2]);
        glRotatef(rotation[0],0,1,0);
        glRotatef(rotation[1],1,0,0);
        glTranslatef(-200,-200,-1000);
        font.Render('8');
        glLoadIdentity();
        gluPerspective(100, 1, 0.05, 10000);
        glTranslatef(observerLoc[0],observerLoc[1],observerLoc[2]);
        glRotatef(rotation[0],0,1,0);
        glRotatef(rotation[1],1,0,0);
        list = np.zeros((1000,3));
        direction = 10;
        for i in range(0,1000):
            direction = direction*-1;
            list[i,:] = [direction,0,-10*i]
        plotTrajectory(list);'''       
        plotFlip();
        pygame.time.wait(10);
        #observerLoc[2] += 0.5;
        if pygame.mouse.get_pressed()[0]:
            if mouseleft == 0:
                mouseleft = 1;
                [prelocX,prelocY]=pygame.mouse.get_pos();
            else:
                [x,y]=pygame.mouse.get_pos();
                rotation[0] -= 0.1*(x-prelocX);
                rotation[1] -= 0.1*(y-prelocY);
                [prelocX,prelocY]=pygame.mouse.get_pos();
        else:
            mouseleft = 0
        if pygame.mouse.get_pressed()[2]:
            if mouseright == 0:
                mouseright = 1;
                [prelocX,prelocY]=pygame.mouse.get_pos();
            else:
                [x,y]=pygame.mouse.get_pos();
                observerLoc[2] -= -1*(y-prelocY);
                [prelocX,prelocY]=pygame.mouse.get_pos();
        else:
            mouseright = 0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit();
                breakflag = 1;
        if breakflag == 1:
            break;
    plotClose();