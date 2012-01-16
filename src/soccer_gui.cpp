/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Kevin J. Walchko.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Kevin  nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Kevin J. Walchko on 6 Mar 2011
 *********************************************************************
 *
 * Status
 * X   Enable simulation capability (random messages)
 * 
 *
 * Change Log:
 *  6 Mar 2011 Created
 *
 **********************************************************************
 *
 * 
 *
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>         // transforms
#include <nav_msgs/Odometry.h>                // odometry
#include <geometry_msgs/Twist.h>              // command and velocity
#include <geometry_msgs/Point.h>              // servo motors
#include <sensor_msgs/Imu.h>                  // IMU messages


#include <OpenGL/gl.h>
#include <OpenGL/glext.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>

#include <string>


// colors
float WHITE[] = {1,1,1,1};
float BLACK[] = {0,0,0,1};
float DARK[] = {.1,.1,.1,1};
float LIGHTGRAY[] = {.8,.8,.8,1};
float LIGHT[] = {.9,.9,.9,1};
float RED[] = {1,0,0,1};
float BLUE[] = {0,0,1,1};
float GREEN[] = {0,1,0,1};
float ORANGE[] = {1,.5,0,1};
float LIGHTBLUE[] = {0,1,1,1};
float PURPLE[] = {1,0,1,1};

// globals used for keeping track of display attributes
int display_size_init = 600; // pixels
									  //int win_width = 1;
									  //int win_height = 1;
									  //float win_pan_x = 0;
									  //float win_pan_y = 0;
float win_aspect_ratio = 1;
float scale_factor = 1; // radius of the size of the word

// globals used for user interaction
int left_pressed = 0;
int right_pressed = 0;
float mouse_pan_x_last;
float mouse_pan_y_last;
float mouse_zoom_y_init;


// global ROS subscriber handles
ros::Subscriber pose_sub;

/*-------------------- basic drawing functions --------------------------*/

int max(int a, int b)
{
	if(a > b)
		return a;
	return b;
}

// draws a rectangle
void drawRectangle(float* pos, float* size, float* color)
{
	glPushMatrix();
	glTranslatef(pos[0],pos[1],pos[2]);
	glScaled(size[0],size[1],1); 
   
	glBegin(GL_POLYGON);
	glColor3f(color[0],color[1],color[2]); 
	glVertex2f(0,1);
	glVertex2f(1,1);
	glVertex2f(1,0);
	glVertex2f(0,0);
	glEnd(); 
   
	glPopMatrix();
}

// draws a rectangle outline
void drawRectangleOutline(float* pos, float* size, float* color)
{
	glPushMatrix();
	glTranslatef(pos[0],pos[1],pos[2]);
	glScaled(size[0],size[1],1); 
   
	glBegin(GL_LINE_STRIP);
	glColor3f(color[0],color[1],color[2]); 
	glVertex2f(0,1);
	glVertex2f(1,1);
	glVertex2f(1,0);
	glVertex2f(0,0);
	glVertex2f(0,1);
	glEnd(); 
   
	glPopMatrix();
}

// draws an arrow from pos1 to pos2
void draw_arrow(float* pos1, float* pos2, float* color)
{
	glColor3f(color[0],color[1],color[2]);
	glBegin(GL_LINE_STRIP);
	glVertex3f(pos1[0],pos1[1],pos1[2]);
	glVertex3f(pos2[0],pos2[1],pos2[2]);
	glEnd();
}

void draw_circle(float* pos, float rad, float* color)
{
	glPushMatrix();
	glTranslatef(pos[0], pos[1], pos[2]);
	glScaled(rad, rad, 1);  
	glColor3f(color[0],color[1],color[2]);
	glBegin(GL_LINE_LOOP); 
	float points = 20;
	for (float i = 0; i < points; i++)
	{    
		float a = 2*M_PI*i/points; 
		glVertex2f(cos(a), sin(a)); 
	} 
	glEnd();
	glPopMatrix();
}

// outputs the text
void draw_string(float* pos, float pad_left, float pad_bottom, const char* string) 
{
	glRasterPos3f(pos[0]+pad_left, pos[1]+pad_bottom, pos[2]);
	for(int i = 0; string[i] != '\0'; i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, string[i]);
}





/*----------------------- GLUT Callbacks --------------------------------*/

// this sets the projection in the current window
void Project()
{
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   //glOrtho(-win_aspect_ratio*scale_factor,+win_aspect_ratio*scale_factor, -scale_factor,+scale_factor, -1,+1);
   glOrtho(-win_aspect_ratio,+win_aspect_ratio, -1,+1, -1,+1);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
}

// default display function, gets called once each loop through glut
void display()
{   
	
   // set up graphics stuff
   Project();
	
   if(1)  
   {  
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);
		glDisable (GL_BLEND);
		glCullFace(GL_BACK);  
		glLoadIdentity();
		glPushMatrix();
		glDepthFunc(GL_ALWAYS);  
		//glTranslatef(win_pan_x,win_pan_y,0);
		
	}
	
	float pos[3] = {0,0,0};
	float arrow_end[3] = {.5,.5,0};
	float size[2] = {.5,.5};
	//drawRectangle(pos,size,WHITE);
	//draw_string(pos,0,0,"hello");
	draw_arrow(pos,arrow_end,WHITE);
	//drawRectangleOutline(pos,size,WHITE);
	draw_circle(pos,.5,WHITE);
	
   // draw the updated scene
   glFlush();
   glutSwapBuffers(); 
}


// this routine is called when the window is resized
void reshape(int width,int height)
{
   //win_width = width;
   //win_height = height;
   //  Ratio of the width to the height of the window
   win_aspect_ratio = (height>0) ? (double)width/height : 1;
   //  Set the viewport to the entire window
   glViewport(0,0, width, height);
   //  Set projection
   Project();
   
   //display_flag = 1;
   glutPostRedisplay();   
}


// this routine gets called when a normal key is pressed
void key(unsigned char ch,int x,int y)
{
	if (ch == 'q' || ch == 'Q')
	{
		exit(0);
	}
	
	Project();
}

// this routine is called when nothing else is happening
void idle()
{
	ros::spinOnce();
	//ROS_INFO("idle()... ");
	glutPostRedisplay(); 
}

// this gets called when glut exits
void cleanup()
{
	ROS_INFO("Exit... ");  
	exit(0);
}

void glutSetup(void){
	
	//  Request double buffered, true color window with Z buffering at 600x600
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(display_size_init,display_size_init);
	glutCreateWindow("Visualization");
	
	//  Set GLUT callbacks
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	//glutSpecialFunc(special);
	glutKeyboardFunc(key);
	
	//glutPassiveMotionFunc(motion);
	//glutMouseFunc(mouse);
	//glutMotionFunc(active_motion);
	glutIdleFunc(idle);
	atexit(cleanup);
	
	glutPostRedisplay();
}




int main(int argc, char *argv[])
{   
	// init ROS stuff  
	ros::init(argc, argv, "test_gl");  
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	ROS_INFO("Start");
	
	//  Initialize GLUT
	glutInit(&argc,argv);
	glutSetup();
	
	//  Pass control to GLUT so it can interact with the user
	glutMainLoop();
	
	return 0;
}



