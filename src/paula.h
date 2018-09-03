#pragma once

// a) read PCD
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "Shlwapi.h" // picky about it's include position --> lib needed, too! 2do: add to CMakeLists.txt

// b) visualize
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


#include <map> // --> map_actor_object

#include "glob_type.h"

#include "paula_cfg.hpp"
#include "paula_cfg_reader.h"


//namespace polo // Pointcloud Object LOcalization  o. PAuLa - [P]ointcloud [Au]to [La]beling
//{
struct s_extcalibration
{
  float32 x;
  float32 y;
  float32 z;
};

s_extcalibration extcali;

enum e_pointcloudfiletype { ft_unknown=0, ft_PCD=1, ft_BIN=2, ft_CSV=3 };
// pointcloud file being loaded
struct s_pointcloudfile
{
  std::string filepath;  // c:/leckere/erdbeermarmelade.honig
  std::string pathname;  // c:/leckere/
  std::string filename;  // erdbeermarmelade.honig
  std::string file_ext;  // .honig
  e_pointcloudfiletype ft;
};

std::string argv_pathname;
s_pointcloudfile pclfile;

pcl::PointCloud<pcl::PointXYZ>::Ptr    p_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr    p_cloud_obst;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud_rgb; // visu

enum e_groundpatchtype { gpt_None, gpt_RANSAC, gpt_Extrapolated };
typedef struct
{
  Eigen::VectorXf coeff; // parametric form of plane: ax + by + cz + d = 0    ...   coeff[0]=a,coeff[1]=b,...[2]=c,...[3]=d
  e_groundpatchtype gp_type;
  std::vector<int> inliers;
// points calculated from a,b,c,d (coeff[0..3]) 
  float fAngleDEG;
  pcl::PointXYZ p1;
  pcl::PointXYZ p2;
  pcl::PointXYZ p3;
  pcl::PointXYZ p4;
  float z_avg;
} s_groundpatch;

s_groundpatch ground_zero; // used as initializer

typedef struct
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  s_groundpatch ground;
} s_cloud_bin;


/* Config */
C_PAuLa_Config paulacfg;

bool b_batch_processing; // do processing + pcl_output (for multiple files), no visualization
bool b_write_label; // save label in Kitty format


std::vector<s_cloud_bin> v_bins;

// ground surface
uint16 bin0;           // ego-vehicle bin / patch(cell)
float32 height_patch0; // ego-vehicle, patch0 height  --> sensor-height ...  2do: set extcali.z with this

// VTK
int text_id;

// Object detection
enum e_objtype { obj_unknown=0, tree=1, car=2, truck=3, guardrail=4, pedestrian=5 }; // <-- put into namespace !!!
//enum e_objtypeKITTY {};

uint16 n_objects;


std::map<std::string, int> map_actor_objdet; // store mapping: actor(string) <--> objectdetections(int), later create a struct "obj"


uint16 xy_to_bin(float32 x, float32 y)
{
  float32 xabs = VELODYNE_MAX_RANGE + x;
  float32 yabs = VELODYNE_MAX_RANGE + y;
  uint16 xBin = xabs / (int)POINTCLOUD_BINWIDTH;
  uint16 yBin = yabs / (int)POINTCLOUD_BINWIDTH;
  uint16 iBin = yBin*N_POINTCLOUD_BINS_X + xBin; // bins: 1d-array
  return iBin;
}


//}
