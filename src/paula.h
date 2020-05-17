#pragma once

#include "Vec3f.hxx"

// a) read PCD
#include <iostream>

#include "Shlwapi.h" // picky about it's include position --> lib needed, too! 2do: add to CMakeLists.txt

// b) visualize


#include <map> // --> map_actor_object

#include "glob_type.h"

// (0) base structures
struct pt2d
{
  float x;
  float y;
};

struct pt3d
{
  float x;
  float y;
  float z;
};

class camera
{
public:
  pt3d rot;
  pt3d trans;
};

//#include "paula_cfg.hpp"
//#include "paula_cfg_reader.h"

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


/* Config */
//C_PAuLa_Config paulacfg;

bool b_batch_processing; // do processing + pcl_output (for multiple files), no visualization
bool b_write_label; // save label in Kitty format


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


//}
