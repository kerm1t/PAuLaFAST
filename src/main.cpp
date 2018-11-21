// ImGui - standalone example application for Glfw + OpenGL 2, using fixed pipeline
// If you are new to ImGui, see examples/README.txt and documentation at the top of imgui.cpp.

#include <imgui.h>
#include "imgui_impl_glfw.h"

#include <stdio.h>


#include "paula.h"
#include "paula_io.hpp" // include before Windows.h (max()), but AFTER! stdlib.h
#include "paula_detect.hpp"
#include "paula_visu.hpp"

#include <GLFW/glfw3.h>

#include <Windows.h>
#include <GL/glu.h>


#include <cmath>
#include <vector>
#include "linmath.h"
#include <time.h>


int pointcloud_load_process(std::string filename);


static void error_callback(int error, const char* description)
{
  fprintf(stderr, "Error %d: %s\n", error, description);
}

#define IM_ARRAYSIZE(_ARR)  ((int)(sizeof(_ARR)/sizeof(*_ARR)))



// a) fill with random data
void pointcloud_random()
{
  srand(time(NULL));
  for (int i = 0; i < 200; i++)
  {
    MeasurementPoint3D pt;
    pt.x = -7.5f + (int)(rand() % 1000) / 1000.0f*15.0f;
    pt.y = -7.5f + (int)(rand() % 1000) / 1000.0f*15.0f;
    pt.z = (int)(rand() % 1000) / 1000.0f*5.0f;
    pt.r = rand() % 200;
    pt.g = rand() % 200;
    pt.b = rand() % 200;
    m_measPoints.push_back(pt);
  }
}

int pointcloud_loadfrom_argv(int argc, char** argv)
{
  if (paulacfg.available())
  {
    std::cout << ".cfg-version = " << paulacfg.cfg_file_vers << std::endl; // read .cfg first, maybe overwritten by commandline parameters
  }
  else
  {
    system("pause");
    return -1;
  }

  if (argc == 1)
  {
    std::cout << "please start with " << PathFindFileName(argv[0]) << " [-b batch][-o output .pcd] <*.pcd|*.bin|*.csv> <*.pcd|*.bin|*.csv> ..." << std::endl;
    return 0;
  }
  else if (argc > 1)
  {
    v_argv.clear();
    for (int i = 1; i < argc; i++)
    {
      std::string sArgv(argv[i]);
      if      (sArgv.compare("-b") == 0) b_batch_processing  = true;
      else if (sArgv.compare("-o") == 0) paulacfg.write_OBST = true;
      else if (sArgv.compare("-l") == 0) b_write_label       = true;
      else v_argv.push_back(sArgv);
    }
  }

  pointcloud_load_process(v_argv[iFile]);


  // get all files in that directory (to browse through them)
  std::string filename = v_argv[iFile];
  std::string s = PathFindFileName(filename.c_str());
  argv_pathname = filename.substr(0, filename.find(s));
  std::cout << argv_pathname << std::endl;
  read_directory(argv_pathname, v_files);
  for (int i = 0; i<v_files.size(); i++)
    std::cout << v_files[i] << std::endl;
}


int pointcloud_load_process(std::string filename) // b) load .pcd
{
  p_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  p_cloud_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>); // output to visu

  if (loadPointCloud(filename, *p_cloud) == -1) return -1;

  m_measPoints.clear();
  
//#define DO_PROCESSING 1
#if (DO_PROCESSING == 1)
  if (process_Pointcloud() == -1) return (-1);// p_cloud, p_cloud_rgb);

  for (int j = 0; j<p_cloud_rgb->points.size(); j++)
  {
    {
      MeasurementPoint3D pt;
      pt.x = p_cloud_rgb->points[j].x;
      pt.y = p_cloud_rgb->points[j].y;
      pt.z = p_cloud_rgb->points[j].z;
      pt.r = p_cloud_rgb->points[j].r;
      pt.g = p_cloud_rgb->points[j].g;
      pt.b = p_cloud_rgb->points[j].b;
      m_measPoints.push_back(pt);
    }
  }
#else
  for (int j = 0; j<p_cloud->points.size(); j++)
  {
    {
      MeasurementPoint3D pt;
      pt.x = p_cloud->points[j].x;
      pt.y = p_cloud->points[j].y;
      pt.z = p_cloud->points[j].z;
      pt.r = 200;
      pt.g = 200;
      pt.b = 200;
      m_measPoints.push_back(pt);
    }
  }
#endif
}

static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
}
static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
  if (button == GLFW_MOUSE_BUTTON_LEFT)
    std::cout << "left mouse clicked" << std::endl;
  if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
    std::cout << "right mouse clicked" << std::endl;
}

// https://www.opengl.org/discussion_boards/showthread.php/126012-converting-window-coordinates-to-world-coordinates
Vec3f Mouse2Dto3D(int x, int y)
{
  GLint viewport[4]; //var to hold the viewport info
  GLdouble modelview[16]; //var to hold the modelview info
  GLdouble projection[16]; //var to hold the projection matrix info
  GLfloat winX, winY, winZ; //variables to hold screen x,y,z coordinates
  GLdouble worldX, worldY, worldZ; //variables to hold world x,y,z coordinates

  glGetDoublev(GL_MODELVIEW_MATRIX, modelview); //get the modelview info
  glGetDoublev(GL_PROJECTION_MATRIX, projection); //get the projection matrix info
  glGetIntegerv(GL_VIEWPORT, viewport); //get the viewport info

  winX = (float)x;
  winY = (float)viewport[3] - (float)y;
//  winZ = 1.0; // The near clip plane is at z = 0 and the far clip plane is at z = 1
  glReadPixels(x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ); // http://nehe.gamedev.net/article/using_gluunproject/16013/

  //get the world coordinates from the screen coordinates
  gluUnProject(winX, winY, winZ, modelview, projection, viewport, &worldX, &worldY, &worldZ);

  Cursor.x = worldX; // <-- OpenGL Cursor
  Cursor.y = worldY;
  Cursor.z = -0.5;// worldZ;
  Cursor.r = 255;
  Cursor.g = 0;
  Cursor.b = 0;

  return Vec3f(worldX, worldY, worldZ);
}

int main(int argc, char** argv)
{
  clock_t begin_time = clock();

  if (argc == 1)
  {
    pointcloud_random();
  }
  else
  {
    pointcloud_loadfrom_argv(argc, argv);
  }


  // Setup window
  glfwSetErrorCallback(error_callback);
  if (!glfwInit())
    return 1;

  GLFWwindow* window = glfwCreateWindow(1280, 720, "Fast Pointcloud AutoLabeling", NULL, NULL);
  if (!window)
  {
    glfwTerminate();
    exit(EXIT_FAILURE);
  }
  glfwMakeContextCurrent(window);

//  glfwSetCursorPosCallback(window, cursor_pos_callback);
//  glfwSetMouseButtonCallback(window, mouse_button_callback); // <-- interferes with IMGui (doesn't work)


  // Setup ImGui binding
  ImGui_ImplGlfw_Init(window, true);



  std::cout << "Loading (+ Processing) + Init Gfx = " << float(clock() - begin_time) / CLOCKS_PER_SEC << " [s]" << std::endl;

  // Main loop
  while (!glfwWindowShouldClose(window))
  {
    glfwPollEvents();


    ImGui_ImplGlfw_NewFrame();
    ImGuiIO& io = ImGui::GetIO(); // using IMGui's Mouse-/Keyb.-states (somehow ImGui seems to block GLfw's callback functions)


    ImGui::RadioButton("none",            &i_processing_step, 0);
    ImGui::RadioButton("RANSAC",          &i_processing_step, 1);
    ImGui::RadioButton("Outlier removal", &i_processing_step, 2);
    ImGui::RadioButton("Clustering",      &i_processing_step, 3);




/*
    // Animate a simple progress bar
    static float progress = 0.0f, progress_dir = 1.0f;
//    if (animate)
    {
      progress += progress_dir * 0.4f * ImGui::GetIO().DeltaTime;
      if (progress >= +1.1f) { progress = +1.1f; progress_dir *= -1.0f; }
      if (progress <= -0.1f) { progress = -0.1f; progress_dir *= -1.0f; }
    }
    // Typically we would use ImVec2(-1.0f,0.0f) to use all available width, or ImVec2(width,0.0f) for a specified width. ImVec2(0.0f,0.0f) uses ItemWidth.
    ImGui::ProgressBar(progress, ImVec2(0.0f, 0.0f));
    ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::Text("RANSAC");

    float progress_saturated = (progress < 0.0f) ? 0.0f : (progress > 1.0f) ? 1.0f : progress;
    char buf[32];
    sprintf(buf, "%d/%d", (int)(progress_saturated * 1753), 1753);
    ImGui::ProgressBar(progress, ImVec2(0.f, 0.f), buf);
    ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::Text("Outliers");
*/



    static int nViewports = 1;
    ImGui::SliderInt("#Viewports", &nViewports, 1, 2);


    if (nViewports == 1) Paula_render_1Viewport(window);
    if (nViewports == 2) Paula_render_2Viewports(window);



    if (io.KeysDown[65] == true) // A
    {
      std::cout << "A pressed" << std::endl;
      if (isPointcloudFileExtension(v_files[iFile])) pointcloud_load_process(argv_pathname + v_files[iFile]);
      if (iFile == v_files.size() - 1) iFile = 0; else iFile++;
    }
/*    if (io.KeysDown[27] == true) // arrow left
    {
      std::cout << "arrow_l pressed" << std::endl;
    }
    if (io.KeysDown[26] == true) // arrow right
    {
      std::cout << "arrow_r pressed" << std::endl;
    }
*/
    if (io.KeysDown[32] == true) // Space
    {
      m_measPoints.clear();

      if (process_Pointcloud() == -1) return (-1);// p_cloud, p_cloud_rgb);
/*      RANSACGrid();
      for (int j = 0; j<p_groundsurface->points.size(); j++)
      {
        pcl::PointXYZRGB prgb(80, 80, 80);
        prgb.x = p_groundsurface->points[j].x;
        prgb.y = p_groundsurface->points[j].y;
        prgb.z = p_groundsurface->points[j].z;
        p_cloud_rgb->push_back(prgb);
      }
*/
      for (int j = 0; j<p_cloud_rgb->points.size(); j++)
      {
        {
          MeasurementPoint3D pt;
          pt.x = p_cloud_rgb->points[j].x;
          pt.y = p_cloud_rgb->points[j].y;
          pt.z = p_cloud_rgb->points[j].z;
          pt.r = p_cloud_rgb->points[j].r;
          pt.g = p_cloud_rgb->points[j].g;
          pt.b = p_cloud_rgb->points[j].b;
          m_measPoints.push_back(pt);
        }
      }

      std::cout << "SPACE pressed" << std::endl;
    }


    if (io.MouseClicked && (io.MouseDown[0] == true) && !io.WantCaptureMouse) // only if not over IMGui GUI-element (https://github.com/ocornut/imgui/issues/52)
    {
      std::cout << "left mouse clicked" << std::endl; 
      std::cout << io.MouseClickedPos[0].x << "," << io.MouseClickedPos[0].y << std::endl;
      Vec3f mouseclicked3d = Mouse2Dto3D(io.MouseClickedPos[0].x, io.MouseClickedPos[0].y);
      std::cout << mouseclicked3d.x << "," << mouseclicked3d.y << "," << mouseclicked3d.z << std::endl;

      mouseFrame.p[0].x = mouseclicked3d.x;
      mouseFrame.p[0].y = mouseclicked3d.y;
      mouseFrame.p[0].z = mouseclicked3d.z;
      mouseFrame.p[0].r = 255;
      mouseFrame.p[0].g = 0;
      mouseFrame.p[0].b = 0;

      Vec3f mouse3d = Mouse2Dto3D(io.MousePos.x, io.MousePos.y);
      std::cout << mouse3d.x << "," << mouse3d.y << "," << mouse3d.z << std::endl;
      mouseFrame.p[1].x = mouseclicked3d.x;
      mouseFrame.p[1].y = mouse3d.y;
      mouseFrame.p[1].z = mouse3d.z;
      mouseFrame.p[1].r = 255;
      mouseFrame.p[1].g = 0;
      mouseFrame.p[1].b = 0;

      mouseFrame.p[2].x = mouse3d.x;
      mouseFrame.p[2].y = mouse3d.y;
      mouseFrame.p[2].z = mouse3d.z;
      mouseFrame.p[2].r = 255;
      mouseFrame.p[2].g = 0;
      mouseFrame.p[2].b = 0;

      mouseFrame.p[3].x = mouse3d.x;
      mouseFrame.p[3].y = mouseclicked3d.y;
      mouseFrame.p[3].z = mouse3d.z;
      mouseFrame.p[3].r = 255;
      mouseFrame.p[3].g = 0;
      mouseFrame.p[3].b = 0;
    }
//    MouseDownPrev = io.MouseDown[0];

    ImGui::Begin("Objects");
    static int selected = -1;
    if (ImGui::TreeNode("classified"))
    {
      ImGui::Columns(3, "mycolumns"); // no. of columns
      ImGui::Separator();
      ImGui::Text("ID"); ImGui::NextColumn();
      ImGui::Text("#Points"); ImGui::NextColumn();
      ImGui::Text("Class"); ImGui::NextColumn();
      ImGui::Separator();
      for (int i = 0; i < v_BBoxes.size(); i++)
      {
        if (v_BBoxes[i].objtype == obj_unknown) continue;
        
        char label[32];
        sprintf(label, "%04d", i);
        if (ImGui::Selectable(label, selected == i, ImGuiSelectableFlags_SpanAllColumns))
          selected = i;

        std::string s_obj;
        if (v_BBoxes[i].objtype == obj_unknown) s_obj = "unknown";
        if (v_BBoxes[i].objtype == truck)       s_obj = "truck";
        if (v_BBoxes[i].objtype == car)         s_obj = "car";
        if (v_BBoxes[i].objtype == guardrail)   s_obj = "guardrail";
        if (v_BBoxes[i].objtype == pedestrian)  s_obj = "ped";

        ImGui::NextColumn();
        ImGui::Text("12"); ImGui::NextColumn();
        ImGui::Text(s_obj.c_str()); ImGui::NextColumn();
        //      ImGui::Text("...."); ImGui::NextColumn();
      }
      ImGui::Columns(1);
      ImGui::Separator();
      ImGui::TreePop();
    }
    ImGui::End();

    // write coordinates of selected obj to vertexarray + colorarray
    if ((v_BBoxes.size() > 0) && (selected >= 0))
    {
      // P0 +--+ P1
      //    |  |
      // P3 +--+ P2
      objSelected.p[0].x = v_BBoxes[selected].AABB.max_point.x;
      objSelected.p[0].y = v_BBoxes[selected].AABB.max_point.y;
      objSelected.p[0].z = 0.0;
      objSelected.p[0].r = 255;
      objSelected.p[0].g = 255;
      objSelected.p[0].b = 0;

      objSelected.p[1].x = v_BBoxes[selected].AABB.max_point.x;
      objSelected.p[1].y = v_BBoxes[selected].AABB.min_point.y;
      objSelected.p[1].z = 0.0;
      objSelected.p[1].r = 255;
      objSelected.p[1].g = 255;
      objSelected.p[1].b = 0;

      objSelected.p[2].x = v_BBoxes[selected].AABB.min_point.x;
      objSelected.p[2].y = v_BBoxes[selected].AABB.min_point.y;
      objSelected.p[2].z = 0.0;
      objSelected.p[2].r = 255;
      objSelected.p[2].g = 255;
      objSelected.p[2].b = 0;

      objSelected.p[3].x = v_BBoxes[selected].AABB.min_point.x;
      objSelected.p[3].y = v_BBoxes[selected].AABB.max_point.y;
      objSelected.p[3].z = 0.0;
      objSelected.p[3].r = 255;
      objSelected.p[3].g = 255;
      objSelected.p[3].b = 0;
    }


    groundplane4picking.p[0].x = 50;
    groundplane4picking.p[0].y = 50;
    groundplane4picking.p[0].z = 0.0;
    groundplane4picking.p[0].r = 222;
    groundplane4picking.p[0].g = 222;
    groundplane4picking.p[0].b = 222;

    groundplane4picking.p[1].x = 50;
    groundplane4picking.p[1].y = -50;
    groundplane4picking.p[1].z = 0.0;
    groundplane4picking.p[1].r = 222;
    groundplane4picking.p[1].g = 222;
    groundplane4picking.p[1].b = 222;

    groundplane4picking.p[2].x = -50;
    groundplane4picking.p[2].y = -50;
    groundplane4picking.p[2].z = 0.0;
    groundplane4picking.p[2].r = 222;
    groundplane4picking.p[2].g = 222;
    groundplane4picking.p[2].b = 222;

    groundplane4picking.p[3].x = -50;
    groundplane4picking.p[3].y = 50;
    groundplane4picking.p[3].z = 0.0;
    groundplane4picking.p[3].r = 222;
    groundplane4picking.p[3].g = 222;
    groundplane4picking.p[3].b = 222;



    ImGui::Render();

    glfwSwapBuffers(window);
  }

  // Cleanup
  ImGui_ImplGlfw_Shutdown();
  glfwTerminate();

  return 0;
}
