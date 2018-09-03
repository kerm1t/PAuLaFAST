#ifndef __PCL_OUT__
#define __PCL_OUT__

#include <pcl/point_types.h>
#include <vector>

#include <pcl/features/moment_of_inertia_estimation.h>
#include "paula.h"

// classification limits:

// GUARDRAIL
#define GUARDRAIL_L_MIN    5.0
#define GUARDRAIL_H_MAX    2.0
#define GUARDRAIL_W_MAX    2.0
#define GUARDRAIL_Z_GROUND 0.75

// TRUCK
#define TRUCK_H_MIN 4.0
#define TRUCK_L_MIN 10.0
#define TRUCK_W_MAX 3.0

// CAR
// small MIN values, as we might see only side / part of car
#define CAR_L_MIN 3.34
#define CAR_L_MAX 5.5
#define CAR_W_MIN 1.19
#define CAR_W_MAX 2.2

// PED
// with NN-clustering of 0.3 + outlier removal
#define PED_H_MIN 1.49 // feet can be associated to the ground --> ped gets less tall
#define PED_H_MAX 2.1
#define PED_W_MIN 0.32
#define PED_W_MAX 1.5
#define PED_Z_GROUND_MIN 0.6
#define PED_Z_GROUND_MAX 1.2


struct s_AABB // axis aligned BBox
{
  pcl::PointXYZ min_point;
  pcl::PointXYZ max_point;
  float32 h; // AUTOSAR: w ... 2do: fix!!
  float32 w; // AUTOSAR: h ... 2do: fix!!
  float32 l;
};

struct s_OBB // oriented BBox
{
  pcl::PointXYZ min_point;
  pcl::PointXYZ max_point;
  pcl::PointXYZ pos;
  float32 h; // AUTOSAR: w ... 2do: fix!!
  float32 w; // AUTOSAR: h ... 2do: fix!!
  float32 l;
  Eigen::Matrix3f rotMAT;
};

// BBox, includes AABB and OBB
struct AA_O_BBox
{
  s_AABB AABB;
  s_OBB   OBB;
  Eigen::Vector3f mass_center;
  e_objtype objtype;
};

std::vector<AA_O_BBox> v_BBoxes;

// --------------------------------------------------------------------------------
// AABB + OBB: http://pointclouds.org/documentation/tutorials/moment_of_inertia.php
// OBB really slow in Debug  15sec --> 3 sec, in Release much faster!
// --> improve with https://github.com/gabyx/ApproxMVBB
// --------------------------------------------------------------------------------
AA_O_BBox Pointcloud_to_BBox(pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud)
{
  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(p_cloud);
  feature_extractor.compute();

  std::vector <float> moment_of_inertia;
  std::vector <float> eccentricity;

  AA_O_BBox box;
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;

  feature_extractor.getMomentOfInertia(moment_of_inertia);
  feature_extractor.getEccentricity(eccentricity);
  feature_extractor.getAABB(box.AABB.min_point, box.AABB.max_point);
  feature_extractor.getOBB(box.OBB.min_point, box.OBB.max_point, box.OBB.pos, box.OBB.rotMAT);
  feature_extractor.getEigenValues(major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter(box.mass_center);
  
  // really "simple" (dummy) obj class. heuristics

  float32 x = box.mass_center(0);
  float32 y = box.mass_center(1);
  float32 z_SENSOR = box.mass_center(2);
  float z_GROUND = z_SENSOR - v_bins[xy_to_bin(x, y)].ground.z_avg;

  float32 h = box.OBB.max_point.y - box.OBB.min_point.y; // AUTOSAR: w ... 2do: fix!!
  float32 w = box.OBB.max_point.z - box.OBB.min_point.z; // AUTOSAR: h ... 2do: fix!!
  float32 l = box.OBB.max_point.x - box.OBB.min_point.x;
  box.OBB.l = l;
  box.OBB.h = h;
  box.OBB.w = w;


  box.objtype = obj_unknown; // init


  if ((h > TRUCK_H_MIN) &&
      (l > TRUCK_L_MIN) &&
      (w < TRUCK_W_MAX)) box.objtype = truck;


  if ((w > CAR_W_MIN) &&
      (w < CAR_W_MAX) &&
      (l > CAR_L_MIN) &&
      (l < CAR_L_MAX)) box.objtype = car;


  if (
     (l > GUARDRAIL_L_MIN) &&
     ((h < GUARDRAIL_H_MAX) && (w < GUARDRAIL_W_MAX)) &&
     (z_GROUND < GUARDRAIL_Z_GROUND)
     ) box.objtype = guardrail;

  float32 _w = box.AABB.max_point.y - box.AABB.min_point.y; // AUTOSAR: w ... 2do: fix!!
  float32 _h = box.AABB.max_point.z - box.AABB.min_point.z; // AUTOSAR: h ... 2do: fix!!
  float32 _l = box.AABB.max_point.x - box.AABB.min_point.x;
  box.AABB.l = _l;
  box.AABB.h = _h;
  box.AABB.w = _w;

  if (
       (_h > PED_H_MIN) && (_h < PED_H_MAX)
    && (_w > PED_W_MIN) && (_w < PED_W_MAX)
    && (_l > PED_W_MIN) && (_l < PED_W_MAX)
    && (z_GROUND > PED_Z_GROUND_MIN) && (z_GROUND < PED_Z_GROUND_MAX)
     ) box.objtype = pedestrian;
  
  return box;
}

#endif // __PCL_OUT__
