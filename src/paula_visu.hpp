#pragma once

#include <GLFW/glfw3.h>

#include <Windows.h>
#include <GL/glu.h>

#include "paula.h"
#include <vector>

pt2d mousepos;
camera cam[2]; // left and right viewport, might be more later

class vehicle
{
public:
  float f_width;
  float f_length;

  vehicle::vehicle() // BMW 5xx, s. http://www.bmw.de/de/neufahrzeuge/5er/touring/2013/technicaldata.html
  {
    f_width = 1.86f;
    f_length = 4.907f;
  }
};

class sensor
{
public:
  ImVec2 pos;       // x,y relative to vehicle center
  float f_rotation; // [rad], 0 = facing to front
  float f_range;    // max. range [m]
  float f_size;

  sensor::sensor()
  {
    f_size = 7.0f;
  }
};

struct MeasurementPoint3D
{
  float x; // [m] fixpoint 2^-7
  float y; // [m]
  float z; // [m]

  unsigned char r;
  unsigned char g;
  unsigned char b;
};
typedef std::vector<MeasurementPoint3D> meas_point_vector;

struct Object2D
{
  struct MeasurementPoint3D p[4];
};

int m_pointSize = 3;
meas_point_vector m_measPoints[2]; // for each viewport
Object2D objSelected;
Object2D groundplane4picking;
MeasurementPoint3D Cursor;
Object2D mouseFrame;



void render(int viewport, int ptsize)
{
  glPushMatrix();

  // b) Velo points to Vertexbuffer (faster than IMMode!)
  //  glScalef(S16Q7_TO_FLOAT, S16Q7_TO_FLOAT, S16Q7_TO_FLOAT);
  glPointSize(float(ptsize));

  glEnableClientState(GL_COLOR_ARRAY);
  glEnableClientState(GL_VERTEX_ARRAY);


  if (m_measPoints[viewport].size() > 0)
  {
    glVertexPointer(3, GL_FLOAT, sizeof(MeasurementPoint3D), &m_measPoints[viewport][0]);
    glColorPointer(3, GL_UNSIGNED_BYTE, sizeof(MeasurementPoint3D), &m_measPoints[viewport][0].r); // r + size(3)
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(m_measPoints[viewport].size()));
  }

  //  if (objSelected...)
/*  {
    glVertexPointer(3, GL_FLOAT, sizeof(MeasurementPoint3D), &objSelected.p[0]);
    glColorPointer(3, GL_UNSIGNED_BYTE, sizeof(MeasurementPoint3D), &objSelected.p[0].r); // r + size(3)
    glDrawArrays(GL_QUADS, 0, static_cast<GLsizei>(4));
  }
  // groundplane 4 picking
  {
    glVertexPointer(3, GL_FLOAT, sizeof(MeasurementPoint3D), &groundplane4picking.p[0]);
    glColorPointer(3, GL_UNSIGNED_BYTE, sizeof(MeasurementPoint3D), &groundplane4picking.p[0].r); // r + size(3)
    glDrawArrays(GL_QUADS, 0, static_cast<GLsizei>(4));
  }
*/  // picking --> drag frame with mouse
  glLineWidth(3.0);
  {
    glVertexPointer(3, GL_FLOAT, sizeof(MeasurementPoint3D), &mouseFrame.p[0]);
    glColorPointer(3, GL_UNSIGNED_BYTE, sizeof(MeasurementPoint3D), &mouseFrame.p[0].r); // r + size(3)
    glDrawArrays(GL_LINE_LOOP, 0, static_cast<GLsizei>(4));
  }

  // Cursor
  glPointSize(12.0);
  {
    glVertexPointer(3, GL_FLOAT, sizeof(MeasurementPoint3D), &Cursor);
    glColorPointer(3, GL_UNSIGNED_BYTE, sizeof(MeasurementPoint3D), &Cursor.r); // r + size(3)
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(1));
  }


  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);

  glPopMatrix();
}

void Paula_render_1Viewport(GLFWwindow* window)
{
//  static float dist = 30.f;
//  ImGui::SliderFloat("dist", &dist, 0.0, 100.0);
//  static float rotspeed = 0;// .007f;
//  ImGui::SliderFloat("rot.speed", &rotspeed, 0.0, 1.0);
  ImGui::SliderInt("pt.size", &m_pointSize, 1, 12);


  glClearColor(0.9, 0.9, 0.9, 1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST); // for glReadPixels --> gluUnproject

                           // Scale to window size
  GLint windowWidth, windowHeight;
  glfwGetWindowSize(window, &windowWidth, &windowHeight);
  glViewport(0, 0, windowWidth, windowHeight);

  // Draw stuff
  glClearColor(0.9, 0.9, 0.9, 1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION_MATRIX);
  glLoadIdentity();
  gluPerspective(60, (double)windowWidth / (double)windowHeight, 0.1, 150);

  glMatrixMode(GL_MODELVIEW_MATRIX); // kein grosser Unterschied, wenn ich das hier hinter die Rot, Trans setze

  glTranslatef(0.0f, 0.0f, -30.0f + cam[0].trans.z);
  glRotatef(         cam[0].rot.x, 1, 0, 0); // rotate once
  glRotatef(180.0f + cam[0].rot.y, 0, 1, 0); // rotate once
  glRotatef( 90.0f + cam[0].rot.z, 0, 0, 1); // rotate once

  render(0, m_pointSize);

  //  alpha += rotspeed;// 0.05;
}

// https://stackoverflow.com/questions/726379/how-to-use-multiple-viewports-in-opengl?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
void Paula_render_2Viewports(GLFWwindow* window)
{
//  static float dist = 30.f;
//  ImGui::SliderFloat("dist", &dist, 0.0, 100.0);
//  static float rotspeed = 0;// .007f;
//  ImGui::SliderFloat("rot.speed", &rotspeed, 0.0, 1.0);
  ImGui::SliderInt("pt.size", &m_pointSize, 1, 12);


  glClearColor(0.9, 0.9, 0.9, 1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Scale to window size
  GLint windowWidth, windowHeight;
  glfwGetWindowSize(window, &windowWidth, &windowHeight);

  // ---------
  // Viewports
  // ---------
  // s. https://stackoverflow.com/questions/726379/how-to-use-multiple-viewports-in-opengl
  // s. auch zu viewports rendern (FBO, ...) https://stackoverflow.com/questions/13710791/multiple-viewports-interfering

  // ----------
  // Viewport 1
  // ----------
  glViewport(0, 0, windowWidth*0.5, windowHeight);
  

  // optional: different background color, s. https://community.khronos.org/t/viewport-background-color/57584/2
  glScissor(0, 0, windowWidth*0.5, windowHeight);
  glEnable(GL_SCISSOR_TEST);
  glClearDepth(1.0);
  glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glDisable(GL_SCISSOR_TEST);
  // optional: different background color


  glMatrixMode(GL_PROJECTION_MATRIX);
  glLoadIdentity();
  gluPerspective(60, (double)windowWidth*0.5 / (double)windowHeight, 0.1, 150);

  glMatrixMode(GL_MODELVIEW_MATRIX);

  glTranslatef(0.0f, 0.0f, -30.0f + cam[0].trans.z);
  glRotatef(cam[0].rot.x, 1, 0, 0); // rotate once
  glRotatef(180.0f + cam[0].rot.y, 0, 1, 0); // rotate once
  glRotatef(90.0f + cam[0].rot.z, 0, 0, 1); // rotate once

  //  glTranslatef(0, 0, -dist);
//  Vec3f eye = Vec3f(0, 0, -dist);// -15);
//  gluLookAt(eye.x, eye.y, eye.z, 0, 0, 0, 0.0, 1.0, 0.0); // Modelview Matrix, s. https://www.opengl.org/archives/resources/faq/technical/viewing.htm

  render(0, 3);


  // ----------
  // Viewport 2
  // ----------
  glViewport(windowWidth*0.5, 0, windowWidth*0.5, windowHeight);

  glMatrixMode(GL_PROJECTION_MATRIX);
  glLoadIdentity();
  gluPerspective(60, (double)windowWidth*0.5 / (double)windowHeight, 0.1, 100);

  glMatrixMode(GL_MODELVIEW_MATRIX);
  //  glTranslatef(0, 0, -dist); // 2do, fix!! x-shift due to Viewports
//  gluLookAt(eye.x, eye.y, eye.z, 0, 0, 0, 0.0, 1.0, 0.0); // Modelview Matrix, s. https://www.opengl.org/archives/resources/faq/technical/viewing.htm


//  static float alpha = 0;
//  glRotatef(alpha, 0, 1, 0); // attempt to rotate cube

  glTranslatef(0.0f, 0.0f, -30.0f + cam[1].trans.z);
  glRotatef(cam[1].rot.x, 1, 0, 0); // rotate once
  glRotatef(180.0f + cam[1].rot.y, 0, 1, 0); // rotate once
  glRotatef(90.0f + cam[1].rot.z, 0, 0, 1); // rotate once

  render(1, m_pointSize);

//  alpha += rotspeed;
}
