#include<windows.h>
#include <GL/glut.h>
#include<bits/stdc++.h>
#include <stdlib.h>
#define rad (3.1416/180)
#define PI 3.1416
#define setUp_size 20
#include "BmpLoader.h"
#include<iostream>
#include<string>

using namespace std;


float zoom=4;
int columnSize[5000][5000]= {0};
float torusX=0,torusY=-2,torusZ=-8;
float torusZ1=-20,torusZ2=-40,torusZ3=-60,torusZ4=-80,torusZ5=-100,torusZ6=-120;
float rotationX=0,rotationY=0,rotationZ=0;

float speed = 0.0;
float angleBackFraction = 0.2;

float Eyex=0.0,Eyey=4.5,Eyez=10.0,Lookx=0,Looky=4,Lookz=0,Upx=0,Upy=1,Upz=0;

int TIME=0;
bool GameStart = false;
float torusPosX[7] = {1,-2,3,-4,-3,0,2};
float torusPosY[7] = {2,3,10,6,7,4,1};

bool rotationStart = false;


unsigned int textureid[6];
int brickwall = 1;
int road = 2;
int balltexture = 3;
int whitewall = 4;
int belun = 5;


void *currentfont;

bool light0_On = true;
bool no_ambient = false;
bool no_diffuse = false;
bool no_specular = false;
bool emmission_on = true;

int XAngle= 0;
int YAngle= 0;
int ZAngle= 0;


float colR=3.0;
float colG=1.5;
float colB=1.0;
float bgColR=0.008;
float bgColG=0.043;
float bgColB=0.059;

float SunY= -5;
float SunZ = -100;
bool Sunflag = 1;
bool normalLight=false;



bool wired=0;
const int L=11;
const int dgre=3;
int ncpt=L+1;
int clikd=0;
const int nt = 40;				//number of slices along x-direction
const int ntheta = 40;

bool isBalloon = false;
int score = 0;
bool gameOver = false;



static void resize(int width, int height)
{
    const float aspectRatio = (float) width / (float) height;

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-aspectRatio, aspectRatio, -1.0, 1.0, 2.0, 1000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}




static GLfloat v_box[8][3] =
{
    {-0.5, 0.0, 0.0},
    {0.5, 0.0, 0.0},
    {-0.5, 0.0, 1.0},
    {0.5, 0.0, 1.0},

    {-0.5, 1.0, 0.0},
    {0.5, 1.0, 0.0},
    {-0.5, 1.0, 1.0},
    {0.5, 1.0, 1.0}
};

static GLubyte quadIndices[6][4] =
{
    {0,1,3,2},
    {0,2,6,4},
    {2,3,7,6},
    {3,1,5,7},
    {1,0,4,5},
    {6,7,5,4}
};

static void getNormal3p
(GLfloat x1, GLfloat y1,GLfloat z1, GLfloat x2, GLfloat y2,GLfloat z2, GLfloat x3, GLfloat y3,GLfloat z3)
{
    GLfloat Ux, Uy, Uz, Vx, Vy, Vz, Nx, Ny, Nz;

    Ux = x2-x1;
    Uy = y2-y1;
    Uz = z2-z1;

    Vx = x3-x1;
    Vy = y3-y1;
    Vz = z3-z1;

    Nx = Uy*Vz - Uz*Vy;
    Ny = Uz*Vx - Ux*Vz;
    Nz = Ux*Vy - Uy*Vx;

    glNormal3f(Nx,Ny,Nz);
}

void unitCube(bool roadTexture=false)
{
    glColor3f(1,1,1);
    //GLfloat no_mat[] = { 0.0, 0.0, 0.0, 1.0 };

    glBegin(GL_QUADS);
    for (GLint i = 0; i <6; i++)
    {
        //glColor3f(colors[4][0],colors[4][1],colors[4][2]);
        getNormal3p(v_box[quadIndices[i][0]][0], v_box[quadIndices[i][0]][1], v_box[quadIndices[i][0]][2],
                    v_box[quadIndices[i][1]][0], v_box[quadIndices[i][1]][1], v_box[quadIndices[i][1]][2],
                    v_box[quadIndices[i][2]][0], v_box[quadIndices[i][2]][1], v_box[quadIndices[i][2]][2]);
        if(roadTexture)
        {
            glVertex3fv(&v_box[quadIndices[i][0]][0]);
            glTexCoord2f(10,10);
            glVertex3fv(&v_box[quadIndices[i][1]][0]);
            glTexCoord2f(10,0);
            glVertex3fv(&v_box[quadIndices[i][2]][0]);
            glTexCoord2f(0,0);
            glVertex3fv(&v_box[quadIndices[i][3]][0]);
            glTexCoord2f(0,10);
        }
        else
        {
            glVertex3fv(&v_box[quadIndices[i][0]][0]);
            glTexCoord2f(1,1);
            glVertex3fv(&v_box[quadIndices[i][1]][0]);
            glTexCoord2f(1,0);
            glVertex3fv(&v_box[quadIndices[i][2]][0]);
            glTexCoord2f(0,0);
            glVertex3fv(&v_box[quadIndices[i][3]][0]);
            glTexCoord2f(0,1);
        }

    }
    glEnd();
    //glutSolidSphere (3.0, 20, 16);

}


void draw_cylinder(GLfloat topRadius,GLfloat baseRadius,GLfloat height,bool baseOpen=false)
{
    GLfloat xb = 0.0;
    GLfloat yb = 0.0;
    GLfloat xt = 0.0;
    GLfloat yt = 0.0;
    GLfloat angle = 0.0;
    GLfloat angle_stepsize = 0.1;


    glColor3f(0.5,0.5,1);
    /** Draw the tube */
    glBegin(GL_QUAD_STRIP);
    angle = 0.0;
    while( angle < 2*PI )
    {
        xb = baseRadius * cos(angle);
        yb = baseRadius * sin(angle);
        xt = topRadius * cos(angle);
        yt = topRadius * sin(angle);
        glVertex3f(xt, yt, height);
        glVertex3f(xb, yb, 0.0);
        angle = angle + angle_stepsize;
    }
    glVertex3f(topRadius, 0.0, height);
    glVertex3f(baseRadius, 0.0, 0.0);
    glEnd();

    /** Draw the circle on top of cylinder */
    glBegin(GL_POLYGON);
    angle = 0.0;
    while( angle < 2*PI )
    {
        xt = topRadius * cos(angle);
        yt = topRadius * sin(angle);
        glVertex3f(xt, yt, height);
        angle = angle + angle_stepsize;
    }
    glVertex3f(topRadius, 0.0, height);
    glEnd();

    glBegin(GL_POLYGON);
    angle = 0.0;
    while( angle < 2*PI )
    {
        xb = baseRadius * cos(angle);
        yb = baseRadius * sin(angle);
        glVertex3f(xb, yb, 0.0);
        angle = angle + angle_stepsize;
    }
    glVertex3f(baseRadius, 0.0, 0.0);
    glEnd();
}



GLfloat ctrlpoints[L+1][3] =
{
    { 0.4, 0.05, 0.0}, { 0.475, 0.925, 0.0},
    { .75, 1.8, 0.0},{ 1.225, 2.4, 0.0},
    {1.75, 2.9, 0.0}, {2.725, 3.25, 0.0},
    {3.25, 3.35, 0.0},{3.975, 2.825, 0.0},
    {4.525, 2.375, 0.0}, {4.80, 2.15, 0.0},
    {5.075, 1.92, 0.0},{5.5,1.5,0.0}

};



class point1
{
public:
    point1()
    {
        x=0;
        y=0;
    }
    int x;
    int y;
} clkpt[2];




long long nCr(int n, int r)
{
    if(r > n / 2) r = n - r; // because C(n, r) == C(n, n - r)
    long long ans = 1;
    int i;

    for(i = 1; i <= r; i++)
    {
        ans *= n - r + i;
        ans /= i;
    }

    return ans;
}



void BezierCurve ( double t,  float xy[2])
{
    double y=0;
    double x=0;
    t=t>1.0?1.0:t;
    for(int i=0; i<=L; i++)
    {
        int ncr=nCr(L,i);
        double oneMinusTpow=pow(1-t,double(L-i));
        double tPow=pow(t,double(i));
        double coef=oneMinusTpow*tPow*ncr;
        x+=coef*ctrlpoints[i][0];
        y+=coef*ctrlpoints[i][1];

    }
    xy[0] = float(x);
    xy[1] = float(y);

    //return y;
}



void setNormal(GLfloat x1, GLfloat y1,GLfloat z1, GLfloat x2, GLfloat y2,GLfloat z2, GLfloat x3, GLfloat y3,GLfloat z3)
{
    GLfloat Ux, Uy, Uz, Vx, Vy, Vz, Nx, Ny, Nz;

    Ux = x2-x1;
    Uy = y2-y1;
    Uz = z2-z1;

    Vx = x3-x1;
    Vy = y3-y1;
    Vz = z3-z1;

    Nx = Uy*Vz - Uz*Vy;
    Ny = Uz*Vx - Ux*Vz;
    Nz = Ux*Vy - Uy*Vx;

    glNormal3f(-Nx,-Ny,-Nz);
}



void balloonBezier()
{
    int i, j;
    float x, y, z, r;				//current coordinates
    float x1, y1, z1, r1;			//next coordinates
    float theta;

    const float startx = 0, endx = ctrlpoints[L][0];
    //number of angular slices
    const float dx = (endx - startx) / nt;	//x step size
    const float dtheta = 2*PI / ntheta;		//angular step size

    float t=0;
    float dt=1.0/nt;
    float xy[2];
    BezierCurve( t,  xy);
    x = xy[0];
    r = xy[1];
    //rotate about z-axis
    float p1x,p1y,p1z,p2x,p2y,p2z;
    for ( i = 0; i < nt; ++i )  			//step through x
    {
        theta = 0;
        t+=dt;
        BezierCurve( t,  xy);
        x1 = xy[0];
        r1 = xy[1];

        //draw the surface composed of quadrilaterals by sweeping theta
        glBegin( GL_QUAD_STRIP );
        for ( j = 0; j <= ntheta; ++j )
        {
            theta += dtheta;
            double cosa = cos( theta );
            double sina = sin ( theta );
            y = r * cosa;
            y1 = r1 * cosa;	//current and next y
            z = r * sina;
            z1 = r1 * sina;	//current and next z

            //edge from point at x to point at next x
            glVertex3f (x, y, z);

            if(j>0)
            {
                setNormal(p1x,p1y,p1z,p2x,p2y,p2z,x, y, z);
            }
            else
            {
                p1x=x;
                p1y=y;
                p1z=z;
                p2x=x1;
                p2y=y1;
                p2z=z1;

            }
            glVertex3f (x1, y1, z1);

            //forms quad with next pair of points with incremented theta value
        }
        glEnd();
        x = x1;
        r = r1;
    } //for i

}




void scoreCheck(){
    float ballPosX = -torusX;
    float ballPosY = -torusY;

    if(torusZ>-8.2 && torusZ<-7.8){
        if((ballPosX>1.5 && ballPosX<4.5) && (ballPosY>8.5 && ballPosY<11.5) ){
            score++;
        }
    }else if(torusZ2>-8.2 && torusZ2<-7.8){
        if((ballPosX>-5.5 && ballPosX<-2.5) && (ballPosY>4.5 && ballPosY<7.5) ){
            score++;;
        }
    }else if(torusZ3>-8.2 && torusZ3<-7.8){
        if((ballPosX>-3.5 && ballPosX<-0.5) && (ballPosY>1.5 && ballPosY<4.5) ){
            score++;
        }
    }else if(torusZ4>-8.2 && torusZ4<-7.8){
        if((ballPosX>-1.5 && ballPosX<1.5) && (ballPosY>2.5 && ballPosY<5.5) ){
            score++;
        }
    }else if(torusZ5>-8.2 && torusZ5<-7.8){
        if((ballPosX>0.5 && ballPosX<3.5) && (ballPosY>-0.5 && ballPosY<2.5) ){
            score++;
        }
    }else if(torusZ6>-8.2 && torusZ6<-7.8){
        if((ballPosX>-5.5 && ballPosX<-2.5) && (ballPosY>2.5 && ballPosY<5.5) ){
            score++;
        }
    }


    if(torusZ>10 && torusZ<10.5){
        if((ballPosX>1.5 && ballPosX<4.5) && (ballPosY>8.5 && ballPosY<11.5) ){
            gameOver=true;
            GameStart=false;
        }
    }else if(torusZ2>10 && torusZ2<10.5){
        if((ballPosX>-5.5 && ballPosX<-2.5) && (ballPosY>4.5 && ballPosY<7.5) ){
            gameOver=true;
            GameStart=false;
        }
    }else if(torusZ3>10 && torusZ3<10.5){
        if((ballPosX>-3.5 && ballPosX<-0.5) && (ballPosY>1.5 && ballPosY<4.5) ){
            gameOver=true;
            GameStart=false;
        }
    }else if(torusZ4>10 && torusZ4<10.5){
        if((ballPosX>-1.5 && ballPosX<1.5) && (ballPosY>2.5 && ballPosY<5.5) ){
            gameOver=true;
            GameStart=false;
        }
    }else if(torusZ5>10 && torusZ5<10.5){
        if((ballPosX>0.5 && ballPosX<3.5) && (ballPosY>-0.5 && ballPosY<2.5) ){
            gameOver=true;
            GameStart=false;
        }
    }else if(torusZ6>10 && torusZ6<10.5){
        if((ballPosX>-5.5 && ballPosX<-2.5) && (ballPosY>2.5 && ballPosY<5.5) ){
            gameOver=true;
            GameStart=false;
        }
    }
}


void drawSun()
{

    glColor3f(colR,colG,colB);
    glutSolidSphere(0.5,30,30);

}


void balloon(float dx,float dy,float dz,float sx,float sy,float sz)
{
    glPushMatrix();
    glTranslatef(dx,dy,dz);
    glScalef(sx,sy,sz);
    glPushMatrix();
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D,textureid[belun]);
    glPushMatrix();
    glTranslatef(0,0.05,0);
    glScalef(0.75,0.75,0.75);
    glTranslatef(0,-0.5,-0.5);
    unitCube();
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);
    glPushMatrix();
    glTranslatef(0.35,0.9,0);
    glScalef(0.05,1,0.05);
    glTranslatef(0,-0.5,-0.5);
    unitCube();
    glPopMatrix();
    glPushMatrix();
    glTranslatef(-0.35,0.9,0);
    glScalef(0.05,1,0.05);
    glTranslatef(0,-0.5,-0.5);
    unitCube();
    glPopMatrix();
    glPushMatrix();
    glColor3f(0.51,0.25,0.32);
    glTranslatef(0,2,0);
    glScalef(0.25,0.25,0.25);
    glRotatef(-90,0,0,1);
    glTranslatef(-3,0,0);
    balloonBezier();
    glPopMatrix();
    glPopMatrix();
    glPopMatrix();

}



void unitWindmill()
{
    double realTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
    double angle = realTime*90.0;
    //base
    glPushMatrix();
    glTranslatef(0,3.1,0);
    glScalef(1,0.08,1);
    glTranslatef(0,-0.5,-0.5);
    unitCube(false);
    glPopMatrix();

    //leg 1
    glPushMatrix();
    glTranslatef(0.45,2.1,0.45);
    glScalef(0.08,2,0.08);
    glTranslatef(0,-0.5,-0.5);
    unitCube(false);
    glPopMatrix();

    //leg2
    glPushMatrix();
    glTranslatef(-0.45,2.1,0.45);
    glScalef(0.08,2,0.08);
    glTranslatef(0,-0.5,-0.5);
    unitCube(false);
    glPopMatrix();

    //leg3
    glPushMatrix();
    glTranslatef(0.45,2.1,-0.45);
    glScalef(0.08,2,0.08);
    glTranslatef(0,-0.5,-0.5);
    unitCube(false);
    glPopMatrix();

    //leg4
    glPushMatrix();
    glTranslatef(-0.45,2.1,-0.45);
    glScalef(0.08,2,0.08);
    glTranslatef(0,-0.5,-0.5);
    unitCube(false);
    glPopMatrix();

    //leg support front
    glPushMatrix();
    glTranslatef(0,2.1,0.45);
    glScalef(1,0.08,0.08);
    glTranslatef(0,-0.5,-0.5);
    unitCube(false);
    glPopMatrix();

    //leg support back
    glPushMatrix();
    glTranslatef(0,2.1,-0.45);
    glScalef(1,0.08,0.08);
    glTranslatef(0,-0.5,-0.5);
    unitCube(false);
    glPopMatrix();

    //motor support 1
    glPushMatrix();
    glTranslatef(0,3.15,-0.40);
    glScalef(0.08,0.1,0.08);
    glTranslatef(0,-0.5,-0.5);
    unitCube(false);
    glPopMatrix();

    //motor support 2
    glPushMatrix();
    glTranslatef(0,3.15,0.40);
    glScalef(0.08,0.1,0.08);
    glTranslatef(0,-0.5,-0.5);
    unitCube(false);
    glPopMatrix();

    //motor
    glPushMatrix();
    glTranslatef(0,3.5,-0.5);
    glScalef(0.2,0.2,0.8);
    glTranslatef(0,-0.5,-0.5);
    draw_cylinder(1,1,2);
    glPopMatrix();

    //fan support
    glPushMatrix();
    glTranslatef(0,3.40,0.77);
    glScalef(0.08,0.08,0.12);
    glTranslatef(0,-0.5,-0.5);
    unitCube(false);
    glPopMatrix();

    //fan blade
    glPushMatrix();
    glPushMatrix();
    glTranslatef(0,3.40,0.82);
    glRotatef(angle,0,0,1);
    glScalef(0.2,2,0.04);
    glTranslatef(0,-0.5,-0.5);
    unitCube(false);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0,3.40,0.82);
    glRotatef(angle,0,0,1);
    glScalef(2,0.2,0.04);
    glTranslatef(0,-0.5,-0.5);
    unitCube(false);
    glPopMatrix();
    glPopMatrix();



}



void singleGreatWall()
{
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D,textureid[brickwall]);
    glPushMatrix();
    glTranslated(0,0,0);
    unitCube(false);
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);
}


void ball()
{
    double realTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
    double angle = realTime*90.0;

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D,textureid[balltexture]);
    glPushMatrix();
    GLUquadric *qobj = gluNewQuadric();
    gluQuadricTexture(qobj, GL_TRUE);
    glRotatef(angle,0,0,1);
    glScalef(0.7,0.7,0.7);
    gluSphere(qobj, 1, 30, 30);
    gluDeleteQuadric(qobj);
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);
}



void greatWall(int n)
{
    for(int i=0; i<n; i++)
    {
        glPushMatrix();
        glTranslated(0,0.2+i,0);
        singleGreatWall();
        glPopMatrix();
    }
}


void windmill()
{
    glPushMatrix();
    glTranslated(-8,-2.0,-5);
    glRotated(65,0,1,0);
    glRotated(15,0,1,0);
    glScaled(2,2,2);
    unitWindmill();
    glPopMatrix();

    glPushMatrix();
    glTranslated(8,-2.0,-5);
    glRotated(-65,0,1,0);
    glRotated(-15,0,1,0);
    glScaled(2,2,2);
    unitWindmill();
    glPopMatrix();
}

void setUp(int n)
{

    /// Ground

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D,textureid[road]);
    glPushMatrix();
    glTranslated(0,0,0);
    glScaled(setUp_size*2,0.3,setUp_size*2);
    unitCube(true);
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);


    //torus
    glColor3d(1,0,0.1);
    glPushMatrix();
    glTranslated(torusPosX[n],torusPosY[n],0);
    glScaled(0.3,0.3,0.3);
    glutSolidTorus(0.1,2,30,30);
    glPopMatrix();

    scoreCheck();
    //whitewall
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D,textureid[whitewall]);
    glPushMatrix();
    glTranslated(torusPosX[n],torusPosY[n],-10);
    glScalef(1.5,1,0.12);
    glTranslatef(0,-0.5,-0.5);
    unitCube(false);
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);

    bool flag = 0;
    for(int i=-(setUp_size/2)+1; i<(setUp_size/2); i+=2)
    {
        for(int j=-(setUp_size/2)+1; j<(setUp_size/2); j+=1)
        {
            if(columnSize[i+(setUp_size/2)+1][j+(setUp_size/2)+1]!=0)
            {
                glPushMatrix();
                glTranslated(i,0,j);
                greatWall(columnSize[i+(setUp_size/2)+1][j+(setUp_size/2)+1]);
                glPopMatrix();
            }
            else if(i>=-5&&i<=5) {}
            else
            {
                if(flag)
                {
                    columnSize[i+(setUp_size/2)+1][j+(setUp_size/2)+1]=(i+(setUp_size/2)+1)%5;
                    flag =0;
                }
                else
                {
                    columnSize[i+(setUp_size/2)+1][j+(setUp_size/2)+1]=(j+(setUp_size/2)+1)%5;
                    flag =1;
                }
                glPushMatrix();
                glTranslated(i,0,j);
                greatWall(columnSize[i+(setUp_size/2)+1][j+(setUp_size/2)+1]);
                glPopMatrix();
            }
        }
    }
}

void draw()
{
    double realTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0;

    TIME = realTime;

    if(rotationX>11)rotationX=11;
    if(rotationX<-11)rotationX=-11;
    if(rotationZ>10)rotationZ=10;
    if(rotationZ<-15)rotationZ=-15;


    glPushMatrix();
    glTranslatef(setUp_size/2,SunY, SunZ);
    drawSun();
    glPopMatrix();
    glPushMatrix();
    glTranslated(0,1,0);
    glRotated(rotationX,1,0,0);
    glRotated(rotationY,0,1,0);
    glRotated(rotationZ,0,0,1);

    glScaled(0.2,0.2,0.2);
    ball();
    glPopMatrix();

    glPushMatrix();
    balloon((setUp_size/2)-2,8,-40,3,3,3);
    balloon(-(setUp_size/2)-2,8,-40,3,3,3);
    glPopMatrix();

    ///setUp
    if(torusX>=4.1)torusX=4.1;
    if(torusX<=-4.1)torusX=-4.1;
    if(torusY>0.1)torusY= 0.1;
    if(torusY<-15)torusY= -15;

    glPushMatrix();
    glTranslated(torusX,torusY,torusZ);
    setUp(2);
    glPopMatrix();

    glPushMatrix();
    glTranslated(torusX,torusY,torusZ1);
    windmill();
    glPopMatrix();

    glPushMatrix();
    glTranslated(torusX,torusY,torusZ2);
    setUp(3);
    glPopMatrix();

    glPushMatrix();
    glTranslated(torusX,torusY,torusZ3);
    setUp(1);
    glPopMatrix();

    glPushMatrix();
    glTranslated(torusX,torusY,torusZ4);
    setUp(5);
    glPopMatrix();

    glPushMatrix();
    glTranslated(torusX,torusY,torusZ5);
    setUp(6);
    glPopMatrix();

    glPushMatrix();
    glTranslated(torusX,torusY,torusZ6);
    setUp(4);
    glPopMatrix();

    torusZ+=speed;
    torusZ1+=speed;
    torusZ2+=speed;
    torusZ3+=speed;
    torusZ4+=speed;
    torusZ5+=speed;
    torusZ6+=speed;

    if(torusZ>=20)torusZ=-110;
    if(torusZ1>=20)torusZ1=-110;
    if(torusZ2>=20)torusZ2=-110;
    if(torusZ3>=20)torusZ3=-110;
    if(torusZ4>=20)torusZ4=-110;
    if(torusZ5>=20)torusZ5=-110;
    if(torusZ6>=20)torusZ6=-110;

    if(rotationX>0)rotationX-=angleBackFraction;
    if(rotationX<0)rotationX+=angleBackFraction;
    if(rotationY>0)rotationY-=angleBackFraction;
    if(rotationY<0)rotationY+=angleBackFraction;
    if(rotationZ>0)rotationZ-=angleBackFraction;
    if(rotationZ<0)rotationZ+=angleBackFraction;


    speed += 0.0002;
    if(speed>=0.1)speed=0.1;
}


void axes()
{
    float length = 4;
    float width = 0.1;

    glPushMatrix();
    // X-axis
    glPushMatrix();
    glTranslatef(length/2-0.05,0,0);
    glScalef(length,width,width);
    glTranslatef(0.0,-0.5,-0.5);
    glColor3f(1,0,0);
    unitCube();  //red
    glPopMatrix();

    // Y-axis
    glPushMatrix();
    glTranslatef(0,length/2-0.05,0);
    glScalef(width,length,width);
    glTranslatef(0.0,-0.5,-0.5);
    glColor3f(0,1,0);
    unitCube();  //green
    glPopMatrix();

    // Z-axis
    glPushMatrix();
    glTranslatef(0,0,length/2-0.05);
    glScalef(width,width,length);
    glTranslatef(0.0,-0.5,-0.5);
    glColor3f(0,0,1);
    unitCube();  //blue
    glPopMatrix();
    glPopMatrix();
}






void lighting()
{
    //light
    GLfloat no_light[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat light_ambient[]  = {0.2, 0.2, 0.2, 1.0};
    GLfloat light_diffuse[]  = { 0.5, 0.5, 0.5, 1.0 };
    GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat light_position0[] = {setUp_size/2,SunY,SunZ};

    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_ambient);
    if(light0_On)
    {
        if(no_ambient)
        {
            glLightfv( GL_LIGHT0, GL_AMBIENT, no_light);
        }
        else
        {
            glLightfv( GL_LIGHT0, GL_AMBIENT, light_ambient);
        }
        if(no_diffuse)
        {
            glLightfv( GL_LIGHT0, GL_DIFFUSE, no_light);
        }
        else
        {
            glLightfv( GL_LIGHT0, GL_DIFFUSE, light_diffuse);
        }
        if(no_specular)
        {
            glLightfv( GL_LIGHT0, GL_SPECULAR, no_light);
        }
        else
        {
            glLightfv( GL_LIGHT0, GL_SPECULAR, light_specular);
        }
    }
    else
    {
        glLightfv( GL_LIGHT0, GL_AMBIENT, no_light);
        glLightfv( GL_LIGHT0, GL_DIFFUSE, no_light);
        glLightfv( GL_LIGHT0, GL_SPECULAR, no_light);
    }
}



void LoadTexture(const char*filename,int i)
{
    glGenTextures(1, &textureid[i]);
    glBindTexture(GL_TEXTURE_2D, textureid[i]);
    glPixelStorei(GL_UNPACK_ALIGNMENT, textureid[i]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    BmpLoader bl(filename);
    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, bl.iWidth, bl.iHeight, GL_RGB, GL_UNSIGNED_BYTE, bl.textureData );
}



void setFont(void *font)
{
    currentfont=font;
}


void drawstring(float x,float y,float z,char *string)
{
    char *c;
    glRasterPos3f(x,y,z);

    for(c=string; *c!='\0'; c++)
    {
        glutBitmapCharacter(currentfont,*c);
    }
}

void MainScreenInstuction()
{
    glPushMatrix();
    setFont(GLUT_BITMAP_TIMES_ROMAN_24);
    drawstring(-1.0,8.0,0.0,"Ball Game ");
    drawstring(-1.4,7.0,0.0,"Press P to Start ");
    glPopMatrix();
}




void GameScreenInstuction()
{
    int finalScore = score/24;
    char numberstring[10];
    itoa(finalScore,numberstring,10);
    glPushMatrix();
    setFont(GLUT_BITMAP_TIMES_ROMAN_24);
    drawstring(-9.0,8.0,0.0,"UP: W, DOWN: S, LEFT: A, RIGHT: D, MAIN MENU: O");
    drawstring(5,8.0,0.0,"Score : ");
    drawstring(6,8.0,0.0,numberstring);
    glPopMatrix();
}


void GameOverInstuction(){
    int finalScore = score/24;
    char numberstring[10];
    itoa(finalScore,numberstring,10);

    glPushMatrix();
    setFont(GLUT_BITMAP_TIMES_ROMAN_24);
    drawstring(-1.4,8.0,0.0,"Game Over ");
    drawstring(-2.4,7.0,0.0,"Total Score is : ");
    drawstring(0,7.0,0.0,numberstring);
    drawstring(-2.4,6.0,0.0,"Press \"O\" to go Home Screen ");
    glPopMatrix();

    speed = 0.0;
    zoom=4;
    torusX=0;
    torusY=-2;
    torusZ=-8;
    torusZ1=-20;
    torusZ2=-40;
    torusZ3=-60;
    torusZ4=-80;
    torusZ5=-100;
    torusZ6=-120;
    rotationX=0;
    rotationY=0;
    rotationZ=0;
    angleBackFraction = 0.2;
    GameStart = false;
    rotationStart = false;
    light0_On = true;
    no_ambient = false;
    no_diffuse = false;
    no_specular = false;
    emmission_on = true;
    XAngle= 0.0;
    YAngle= 0.0;
    ZAngle= 0.0;
    colR=3.0;
    colG=1.5;
    colB=1.0;
    bgColR=0.008;
    bgColG=0.043;
    bgColB=0.059;
    SunY= -5;
    SunZ = -100;
    Sunflag = 1;
    wired=0;
    isBalloon = false;
    normalLight =false;
}





static void display(void)
{
    const double realTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
    double angle = realTime*90.0;
    double extraAngle=angle;

    if(!rotationStart)
    {
        angle=0;
    }

    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glClearColor(bgColR,bgColG,bgColB,0.0);

    glLoadIdentity();

    gluLookAt(	Eyex, Eyey, Eyez,
                Lookx, Looky, Lookz,
                Upx, Upy, Upz);


    if(wired)
    {
        glPolygonMode( GL_FRONT, GL_LINE ) ;
        glPolygonMode( GL_BACK, GL_LINE ) ;

    }
    else
    {
        glPolygonMode( GL_FRONT,GL_FILL ) ;
        glPolygonMode( GL_BACK, GL_FILL ) ;
    }

    glRotatef(YAngle, 0,1,0);
    glRotatef(XAngle, 1,0,0);
    glRotatef(ZAngle, 0,0,1);
    //axes();

    glPushMatrix();
    lighting();
    glPopMatrix();
    if(GameStart)
    {
        glPushMatrix();
        glTranslated(0,0,0);
        glScaled(zoom,zoom,zoom);
        glRotated(angle,0,1,0);
        draw();
        glPopMatrix();

        GameScreenInstuction();
    }else if(gameOver){
        GameOverInstuction();
    }
    else
    {
        if(isBalloon)
        {
            balloon(0,0,0,2,2,2);
        }
        else
        {
            glPushMatrix();
            glTranslated(0,3,0);
            glRotated(extraAngle,0,1,0);
            glScaled(1.5,1.5,1.5);
            ball();
            glPopMatrix();

            MainScreenInstuction();
        }
    }

    glutSwapBuffers();
}





void update(int value)
{
    if(light0_On)
    {
        if(normalLight)
        {
            SunZ = -100;
            SunY = -5;
            Sunflag=1;
            colR=2.0;
            colG=1.50;
            colB=1.0;
            bgColR=0.53;
            bgColG=0.81;
            bgColB=0.92;
        }
        else
        {
            if(SunZ>5)
            {
                SunZ = -100;
                SunY = -5;
                Sunflag=1;
                colR=2.0;
                colG=1.50;
                colB=1.0;

                bgColR=0.008;
                bgColG=0.043;
                bgColB=0.059;
            }

            if(Sunflag)
            {
                SunZ += 0.5;
                colR-=0.001;
                //colG+=0.002;
                colB+=0.005;

                bgColR+=0.004;
                bgColG+=0.007;
                bgColB+=0.008;

                if(SunY<7)
                {
                    SunY += 0.5;
                }
                else if(SunZ>-7)
                {
                    Sunflag=0;
                }
            }
            else
            {
                SunZ += 0.5;
                SunY -= 0.5;
                colR+=0.001;
                colB-=0.01;

                bgColR-=0.004;
                bgColG-=0.007;
                bgColB-=0.008;

                if(SunY<-5)
                {
                    Sunflag=1;
                }
            }
        }
    }
    else
    {
        SunZ = -100;
        SunY = -5;
        Sunflag=1;
        colR=2.0;
        colG=1.50;
        colB=1.0;
        bgColR=0.008;
        bgColG=0.043;
        bgColB=0.059;

    }

    glutPostRedisplay();
    glutTimerFunc(25, update, 0);
}


static void key(unsigned char key, int x, int y)
{
    float fraction = 0.3;
    float rotationFraction = 1;
    switch (key)
    {
    case 27 :
    case 'Q':
    case 'q':
        exit(0);
        break;
    case 'R':
    case 'r':
        rotationStart=true;
        break;
    case 'T':
    case 't':
        rotationStart=false;
        break;
    case 'z':
        zoom+=0.05;
        break;
    case 'Z':
        zoom-=0.05;
    case 'W':
    case 'w':
        torusY-=fraction;
        rotationZ+=rotationFraction;
        break;
    case 'S':
    case 's':
        torusY+=fraction;
        rotationZ-=rotationFraction;
        break;
    case 'A':
    case 'a':
        torusX+=fraction;
        rotationX-=rotationFraction*3;
        rotationY+=rotationFraction/2;
        break;
    case 'D':
    case 'd':
        torusX-=fraction;
        rotationX+=rotationFraction*3;
        rotationY-=rotationFraction/2;
        break;
    case 'P':
    case 'p':
        score=0;
        GameStart=true;
        break;
    case 'O':
    case 'o':
        score=0;
        GameStart=false;
        gameOver = false;
        break;
    case 'B':
        XAngle =((--XAngle)%360);
        break;
    case 'b':
        XAngle = (++XAngle)%360;
        break;
    case 'N':
        YAngle = (YAngle-1)%360;
        break;
    case 'n':
        YAngle = (YAngle+1)%360;
        break;
    case 'M':
        ZAngle = (ZAngle-1)%360;
        break;
    case 'm':
        ZAngle = (ZAngle+1)%360;
        break;
    case '1':
        no_ambient = !no_ambient;
        break;
    case '2':
        no_diffuse = !no_diffuse;
        break;
    case '3':
        no_specular = !no_specular;
        break;
    case '4':
        light0_On = !light0_On;
        break;
    case '5':
        normalLight = !normalLight;
        break;
    case '0':
        wired = !wired;
        break;
    case '9':
        isBalloon = !isBalloon;
        break;
    case 'Y':
    case 'y':
        zoom=4;
        torusX=0;
        torusY=-2;
        torusZ=-8;
        torusZ1=-20;
        torusZ2=-40;
        torusZ3=-60;
        torusZ4=-80;
        torusZ5=-100;
        torusZ6=-120;
        rotationX=0;
        rotationY=0;
        rotationZ=0;
        speed = 0.0;
        angleBackFraction = 0.2;
        GameStart = false;
        rotationStart = false;
        light0_On = true;
        no_ambient = false;
        no_diffuse = false;
        no_specular = false;
        emmission_on = true;
        XAngle= 0.0;
        YAngle= 0.0;
        ZAngle= 0.0;
        colR=3.0;
        colG=1.5;
        colB=1.0;
        bgColR=0.008;
        bgColG=0.043;
        bgColB=0.059;
        SunY= -5;
        SunZ = -100;
        Sunflag = 1;
        wired=0;
        isBalloon = false;
        normalLight =false;
        score= 0;
        gameOver = false;
        break;
    }

    glutPostRedisplay();
}

static void idle(void)
{
    glutPostRedisplay();
}


/* Program entry point */

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitWindowPosition(0,0);
    glutInitWindowSize(1366,720);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGBA);

    glutCreateWindow("1607005");

    glutReshapeFunc(resize);
    glutDisplayFunc(display);
    glutKeyboardFunc(key);
    glutIdleFunc(idle);


    glClearColor(bgColR,bgColG,bgColB,1); //0.53,0.81,0.92,1
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glutTimerFunc(25, update, 0);


    LoadTexture("D:\\Programming\\study\\4-2\\lecture\\CSE 4208 Graphics lab\\assignment\\project\\bricks.bmp",brickwall);
    LoadTexture("D:\\Programming\\study\\4-2\\lecture\\CSE 4208 Graphics lab\\assignment\\project\\brick2.bmp",whitewall);
    LoadTexture("D:\\Programming\\study\\4-2\\lecture\\CSE 4208 Graphics lab\\assignment\\project\\road.bmp",road);
    LoadTexture("D:\\Programming\\study\\4-2\\lecture\\CSE 4208 Graphics lab\\assignment\\project\\ball2.bmp",balltexture);
    LoadTexture("D:\\Programming\\study\\4-2\\lecture\\CSE 4208 Graphics lab\\assignment\\project\\belun.bmp",belun);

    cout<<"Instruction :"<<endl;
    cout<<"Press w to go up"<<endl;
    cout<<"Press s to go down"<<endl;
    cout<<"Press a to go left"<<endl;
    cout<<"Press d to go right"<<endl;
    cout<<"Press r to start rotation"<<endl;
    cout<<"Press t to stop rotation"<<endl;
    cout<<"Press z to Zoom"<<endl;
    cout<<"Press p to start game"<<endl;
    cout<<"Press o to go Home"<<endl;
    cout<<"Press 1 to toggle ambient"<<endl;
    cout<<"Press 2 to toggle diffuse"<<endl;
    cout<<"Press 3 to toggle specular"<<endl;
    cout<<"Press 4 to toggle Sun light"<<endl;
    cout<<"Press 5 to toggle night mode"<<endl;
    cout<<"Press 0 to toggle wired mode"<<endl;
    cout<<"Press 9 to toggle balloon in home "<<endl;
    cout<<"Press y to reset "<<endl;

    glutMainLoop();

    return EXIT_SUCCESS;
}
