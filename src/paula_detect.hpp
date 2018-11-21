#ifndef __PAULA_PROCESS__
#define __PAULA_PROCESS__

// c) RANSAC
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

// d) Outlier removal
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

// e) Segment
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/extract_indices.h>

// OBB
#include <pcl/features/moment_of_inertia_estimation.h>


#include "paula.h"
#include "paula_cfg.hpp"
#include "paula_classify.hpp"
#include "paula_io.hpp"

#include "Vec3f.hxx"


int i_processing_step;


#ifdef FREESPACE
#include "frs_customer_defs.h"
#include "frs_point.h"

FRS_INT_1B0_t_Freespace fFreespace;
FRS_FreeSpaceFanDef fFanDef;
#endif


void ground_patches_neighbours();
void calc_ground_patches_P1P4();
void calc_ground_patches_P1P4_Extr(); // ...ugly!!!


pcl::PointCloud<pcl::PointXYZ>::Ptr p_groundsurface;   // detected by RANSAC
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> v_obstacles_RANSAC;
pcl::PointCloud<pcl::PointXYZ>::Ptr p_obstacles_OTHER; // rest --> Clustering


s_groundpatch fitplane_RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud)
{
  s_groundpatch plane;

  // create RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(p_cloud));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
  ransac.setDistanceThreshold(paulacfg.RANSAC_DISTTHRESH);
  /*
  Example: "Werkstatt" 45129 points
  Threshold     [ms]   #points
  0.01        36.375     7094
  0.05         4.063    14716
  0.08         3.746    15162
  0.12         3.422    15607

  Example: "Werkstatt" 65xxx points
  Threshold     [ms]   #points
  0.07        40.xxx    12xxx
  */
  ransac.computeModel();
  ransac.getInliers(plane.inliers);
  if (plane.inliers.size() > GROUNDSURFACE_MINPOINTS)
    // doesn't work -->  if (plane.coeff.size() > 0)
  {
    // http://www.pointclouds.org/assets/files/presentations/ICCV2011-segmentation.pdf
    ransac.getModelCoefficients(plane.coeff);
  }
  return plane;
  //  return 0;
}


// PointcloudSrc, Indices --> PointcloudDst
void outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloudSrc, std::vector<int> * indices, pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloudDst)
{
  pcl::IndicesPtr p_indices = boost::make_shared<std::vector<int> >(*indices);
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Extract the inliers
  extract.setInputCloud(p_cloudSrc);
  extract.setIndices(p_indices);
  extract.setNegative(true);
  extract.filter(*p_cloudDst);
}


int RANSACGrid()
{
  clock_t begin_time = clock();

  std::cout << "Ransac-->" << std::endl;

  // ==== RANSAC ====>
  begin_time = clock();


  p_groundsurface.reset  (new pcl::PointCloud<pcl::PointXYZ>);
  p_obstacles_OTHER.reset(new pcl::PointCloud<pcl::PointXYZ>);


  v_bins.clear(); // vorher die Pointcloud löschen?

  std::cout << "init bin clouds-->" << std::endl;
  // init bin clouds
  for (int iBin = 0; iBin<N_POINTCLOUD_BINS; iBin++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    s_cloud_bin bin;
    bin.cloud = p_cloud;
    bin.ground = ground_zero; // bringt nix, da wir noch eine Vergleicher Funktion brauchen 
    v_bins.push_back(bin);
  }
  // fill points into bins
  std::cout << "fill points into bins-->" << std::endl;
  for (int iSrc = 0; iSrc<p_cloud->points.size(); iSrc++)
  {
    float32 x = VELODYNE_MAX_RANGE + p_cloud->points[iSrc].x;
    float32 y = VELODYNE_MAX_RANGE + p_cloud->points[iSrc].y;
    int xBin = x / (int)POINTCLOUD_BINWIDTH;
    int yBin = y / (int)POINTCLOUD_BINWIDTH;
    int iBin = yBin*N_POINTCLOUD_BINS_X + xBin; // bins: 1d-array
    if ((iBin >= 0) && (iBin < N_POINTCLOUD_BINS))
    {
      pcl::PointXYZ p = p_cloud->points[iSrc];
      if ((abs(p.x) < fEGO_REFL_SUPRESS_X_SIDE) &&
        (abs(p.y) < fEGO_REFL_SUPRESS_Y_SIDE)) // omit ego-vehicle
      {
        pcl::PointXYZRGB prgb(255, 255, 0);
        prgb.x = p.x;
        prgb.y = p.y;
        prgb.z = p.z;
        p_cloud_rgb->push_back(prgb);
      }
      else
      {
        v_bins[iBin].cloud->points.push_back(p);
      }
    }
  }

  // each bin -> RANSAC
  std::cout << "each bin -> RANSAC-->" << std::endl;
  for (int iBin = 0; iBin<N_POINTCLOUD_BINS; iBin++)
  {
    v_bins[iBin].ground.gp_type = gpt_None; // init

    // -------------------------------------------------------------------
    // for debugging: filter all Bins except the one denoted in the config
    // -------------------------------------------------------------------
    if ((paulacfg.iBIN_FILTER > 0) && (iBin != paulacfg.iBIN_FILTER)) continue;

    uint32 nSize = v_bins[iBin].cloud->points.size();
    if (nSize > 3)
    {
      s_groundpatch plane;
      int n_runs = 0;

      // check normal vector of plane facing up/down
      // repeat till plane with normal of min(z) found --> ~ groundplane
      while (
        (n_runs == 0) ||
        ((plane.coeff.size() > 3) && (abs(plane.coeff[2]) < paulacfg.GROUNDSURFACE_Z_NORMAL_MIN))
        )
      {
        plane = fitplane_RANSAC(v_bins[iBin].cloud);
        n_runs++;
        //        if (plane.n_runs == 204) break; // HACK, sometimes inf.loop here
        if (plane.coeff.size() > 3)
        {
          float fAngle = planeAngleRAD(Vec3f(plane.coeff[0], plane.coeff[1], plane.coeff[2]))*180.0 / M_PI;
          plane.inliers.size();
          std::cout << "..bin #" << iBin << ", " << plane.inliers.size() << " pts, " << fAngle << ", N.z " << plane.coeff[2] << ", " << std::endl;
        }

        if ((plane.coeff.size() > 3) && (abs(plane.coeff[2]) < paulacfg.GROUNDSURFACE_Z_NORMAL_MIN))
        {

          // probably not a ground plane ... but we can use this as an object detected --> move points to obstacle (currently by colouring)
          pcl::PointCloud<pcl::PointXYZ>::Ptr p_obstacles_RANSAC(new pcl::PointCloud<pcl::PointXYZ>);

          //          pcl::PointXYZ p2; // RANSAC obstacles

          for (int i = 0; i<plane.inliers.size(); i++)
          {
            pcl::PointXYZ p = v_bins[iBin].cloud->points[plane.inliers[i]];
            //            if ((abs(p.x) < EGO_REFL_SUPRESS_X_SIDE) &&
            //                (abs(p.y) < EGO_REFL_SUPRESS_Y_SIDE)) // omit ego-vehicle
            //              ; else {
            p_obstacles_RANSAC->points.push_back(p);
            p_obstacles_OTHER->points.push_back(p); // much better than RANSAC parts! but takes additional time
                                                    ///           if (plane.inliers.size() > 20) // avoid e.g. small reflections, 2do: around for(...) loop
                                                    //              {
                                                    //                p_obstacles_FRS->points.push_back(p);
                                                    //              }
                                                    //            }
          }

          v_obstacles_RANSAC.push_back(p_obstacles_RANSAC);
          // --> compute RANSAC again on remaining points
          pcl::PointCloud<pcl::PointXYZ>::Ptr p_outliers2(new pcl::PointCloud<pcl::PointXYZ>);
          outliers(v_bins[iBin].cloud, &plane.inliers, p_outliers2);
          copyPointCloud(*p_outliers2, *v_bins[iBin].cloud);  //// AAAchtung !!!
        }
      }

      if (plane.coeff.size() > 3)
      {
        v_bins[iBin].ground.fAngleDEG = planeAngleRAD(Vec3f(plane.coeff[0], plane.coeff[1], plane.coeff[2]))*180.0 / M_PI;

        std::cout << "groundpatch in bin #" << iBin << ", " << v_bins[iBin].ground.fAngleDEG << "; "
          << plane.coeff[0] << ", " << plane.coeff[1] << ", " << plane.coeff[2] << ", " << plane.coeff[3] << "." << std::endl;
        v_bins[iBin].ground.coeff = plane.coeff;
        v_bins[iBin].ground.gp_type = gpt_RANSAC;
        // inliers(bin) --> groundsurface
        for (int i = 0; i < plane.inliers.size(); i++)
        {
          pcl::PointXYZ p = v_bins[iBin].cloud->points[plane.inliers[i]];
          p_groundsurface->points.push_back(p);
        }
      }

      // get "outliers" per bin --> obstacles
      pcl::PointCloud<pcl::PointXYZ>::Ptr p_outliers(new pcl::PointCloud<pcl::PointXYZ>);
      outliers(v_bins[iBin].cloud, &plane.inliers, p_outliers);
      pcl::PointXYZ p2; // other obstacles
      for (int i = 0; i<p_outliers->points.size(); i++)
      {
        pcl::PointXYZ p = p_outliers->points[i];
        //        if ((abs(p.x) < fEGO_REFL_SUPRESS_X_SIDE) &&
        //            (abs(p.y) < fEGO_REFL_SUPRESS_Y_SIDE)) // omit ego-vehicle
        //          ; else {
        p_obstacles_OTHER->points.push_back(p);
        // might contain "crap" do not include in FRS pointcloud, instead fill FRS pointcloud after Clustering
        //        }
      }
    }
  }
  std::cout << "RANSAC-Grid " << float(clock() - begin_time) / CLOCKS_PER_SEC << " [s] ___ "
    << "groundsurface has " << (uint32)p_groundsurface->points.size() << " data points" << std::endl;
  // <==== RANSAC ====

  return 1;
}


int process_Pointcloud() // global params: pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud_rgb
{
  const pcl::PointCloud<pcl::PointXYZ>::Ptr p_obstacles_FRS(new pcl::PointCloud<pcl::PointXYZ>);

  clock_t begin_time = clock();

  RANSACGrid();

  // -----------------------------------------------------------------------------
  // RANSAC "POST-Processing"
  // 1) Remove Ground patches that height – height (neighbour-bin) > t2               --> now!
  // 2) Add objects to ground surface, that are center(obj) – height(bin(obj) < t1    --> after clustering
  // -----------------------------------------------------------------------------

  // as RANSAC is done per gridcell, RANSAC (obstacles) are split --> thus merge again with others and give to clustering
  for (int j = 0; j<p_groundsurface->points.size(); j++)
  {
    pcl::PointXYZRGB prgb(80, 80, 80);
    prgb.x = p_groundsurface->points[j].x;
    prgb.y = p_groundsurface->points[j].y;
    prgb.z = p_groundsurface->points[j].z;
    p_cloud_rgb->push_back(prgb);
  }

  for (int i = 0; i < v_obstacles_RANSAC.size(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    unsigned char r = 255; // orange
    unsigned char g = 165;
    unsigned char b = 0;
    for (int j = 0; j<v_obstacles_RANSAC[i]->points.size(); j++)
    {
      pcl::PointXYZRGB prgb(r, g, b);
      prgb.x = v_obstacles_RANSAC[i]->points[j].x;
      prgb.y = v_obstacles_RANSAC[i]->points[j].y;
      prgb.z = v_obstacles_RANSAC[i]->points[j].z;
      p_cloud_rgb->push_back(prgb);
    }
  }

  p_obstacles_OTHER->width = p_obstacles_OTHER->size();  // <-- that's the fix !!!
  p_obstacles_OTHER->height = 1;  // <-- that's the fix !!!
  if (paulacfg.write_OBST)
  {
    // 2do: write e.g. on keystroke
    std::stringstream stream;
//    stream << pclfile.pathname << pclfile.filename << "_obst" << pclfile.file_ext; // same folder -> filename appendix "_obst"
    stream << pclfile.pathname << "obst\\" << pclfile.filename << pclfile.file_ext; // subfolder
    pcl::io::savePCDFileASCII(stream.str(), *p_obstacles_OTHER);
    std::cerr << "Saved " << p_obstacles_OTHER->points.size() << " data points to " << stream.str() << std::endl;
  }

  if (b_batch_processing) return (0); // hack!


  if (paulacfg.b_REM_OUTLIERS)
  {
    // doesn't work yet... http://pointclouds.org/documentation/tutorials/remove_outliers.php
    pcl::PointCloud<pcl::PointXYZ>::Ptr filt_in(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(*p_obstacles_OTHER, *filt_in);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filt_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(filt_in);
    outrem.setRadiusSearch(paulacfg.OUTLIERS_RADIUS);
    outrem.setMinNeighborsInRadius(paulacfg.OUTLIERS_NEIGHBOURS_MIN);
    // apply filter
    outrem.filter(*filt_out);
    copyPointCloud(*filt_out, *p_obstacles_OTHER);
  }


  calc_ground_patches_P1P4();
  
  ground_patches_neighbours();
  
  calc_ground_patches_P1P4_Extr();

  
  // remove ground points from obstacles
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  for (int i = 0; i < (*p_obstacles_OTHER).size(); i++)
  {
    pcl::PointXYZ pt(p_obstacles_OTHER->points[i].x, p_obstacles_OTHER->points[i].y, p_obstacles_OTHER->points[i].z);
    int iBin = xy_to_bin(pt.x, pt.y);
    float zAvg = v_bins[iBin].ground.z_avg;

    if (abs(pt.z - zAvg) < 0.10)
    {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(p_obstacles_OTHER);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*p_obstacles_OTHER); // on itself


  // Freespace --> after clustering (outliers are not clustered)


  // clustering
  // http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction
  if (paulacfg.b_DO_CLUSTERING)
  {
    clock_t begin_time_CLU = clock();

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    std::vector<pcl::PointIndices> v_idx_cluster;
    // ------------------------------------------------------------------------------------
    // CLUSTER_TOLERANCE:
    // 30cm --> Objekte in der Naehe werden gut getrennt, aber weiter entfernte "zerfallen"
    // 50cm --> weiter entfernte Objekte werden gut zusammengefasst
    // 2do: vary clusterdistance with dist from ego-vehicle
    // ------------------------------------------------------------------------------------
    ec.setClusterTolerance(paulacfg.CLUSTER_TOLERANCE);
    ec.setMinClusterSize(CLUSTER_MINCOUNT);
    ec.setMaxClusterSize(CLUSTER_MAXCOUNT);
    ec.setInputCloud(p_obstacles_OTHER);
    ec.extract(v_idx_cluster);

    pcl::ExtractIndices<pcl::PointXYZ> EXI;
    for (int i = 0; i < v_idx_cluster.size(); ++i)
    {
      begin_time = clock();
      pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);

      EXI.setInputCloud(p_obstacles_OTHER); // p_filtered / p_obstacles
      EXI.setIndices(boost::make_shared<const pcl::PointIndices>(v_idx_cluster[i]));
      EXI.filter(*object_cloud);
      
      v_BBoxes.push_back(Pointcloud_to_BBox(object_cloud)); // classification
      std::cout << "Box " << (i + 1) << "/" << v_idx_cluster.size() << ", " << object_cloud->points.size() << " pts, " << float(clock() - begin_time) / CLOCKS_PER_SEC << " [s]. " << std::endl;

      unsigned char r = rand() % 255;
      unsigned char g = rand() % 255;
      unsigned char b = rand() % 255;
      for (int j = 0; j<object_cloud->points.size(); j++)
      {
        pcl::PointXYZRGB prgb(r, g, b);
        prgb.x = object_cloud->points[j].x;
        prgb.y = object_cloud->points[j].y;
        prgb.z = object_cloud->points[j].z;
        p_cloud_rgb->push_back(prgb);

//        if (object_cloud->points.size() > 20) // avoid e.g. small reflections
        {
          pcl::PointXYZ p; // 2do: cast prgb
          p.x = object_cloud->points[j].x;
          p.y = object_cloud->points[j].y;
          p.z = object_cloud->points[j].z;
          p_obstacles_FRS->points.push_back(p);
        }
      }
    }
    std::cout << "Clustering " << float(clock() - begin_time_CLU) / CLOCKS_PER_SEC << " [s] ___ " << v_idx_cluster.size() << " clusters found." << std::endl;
  }
  else
  { // no clustering --> all non-ground points
    for (int j = 0; j < p_obstacles_OTHER->points.size(); j++)
    {
      pcl::PointXYZRGB prgb(200, 200, 200);
      prgb.x = p_obstacles_OTHER->points[j].x;
      prgb.y = p_obstacles_OTHER->points[j].y;
      prgb.z = p_obstacles_OTHER->points[j].z;
      p_cloud_rgb->push_back(prgb);

      pcl::PointXYZ p;                   // 2do: cast prgb
      p.x = p_obstacles_OTHER->points[j].x;
      p.y = p_obstacles_OTHER->points[j].y;
      p.z = p_obstacles_OTHER->points[j].z;
      p_obstacles_FRS->points.push_back(p);
    }
  }


  // RANSAC "POST-Processing" 2)
  // 2) Add objects to ground surface, that are center(obj) – height(bin(obj) < t1    --> after clustering
  std::cout << "PP(2): " << v_BBoxes.size() << " Boxes before." << std::endl;
  std::vector<AA_O_BBox>::iterator it = v_BBoxes.begin();
  for (; it != v_BBoxes.end();) // use iterator for erasing from std::vector
  {
    float32 x = it->mass_center(0);
    float32 y = it->mass_center(1);
    float32 z = it->mass_center(2);
    uint16 iBin = xy_to_bin(x, y);
    if (v_bins[iBin].ground.gp_type != gpt_None)
    {
      float32 zground = v_bins[iBin].ground.z_avg;
      std::cout << z << "|" << zground << std::endl;
      if (abs(abs(z) - abs(zground)) < PPROC_GROUNDOBJ_TOL_Z) // whacky <-- 2do: get rid of abs(abs(abs... & 2do: ground tiles smoothneess constraint!!!
      {
        it = v_BBoxes.erase(it); // it needed, otherwise using for loop, we get a crash
        std::cout << "erasing " << z << "|" << zground << std::endl;
      }
      else
        ++it;
    }
    else
      ++it;
  }
  std::cout << "PP(2): " << v_BBoxes.size() << " Boxes after." << std::endl;

#ifdef FREESPACE
  // ===== BMW freespace =====
  if (paulacfg.b_DO_FREESPACE)
  {
    begin_time = clock();

    for (uint32 ui = 0; ui < FRS_NUM_SEGMENTS; ui++)
    {
      fFreespace.a_Segment[ui].b_DistanceDefined = FALSE;
      fFreespace.a_Segment[ui].a_Obstacle[0].fDistance = FRS_DEFAULT_INITIAL_DISTANCE_VELODYNE;
      fFreespace.a_Segment[ui].a_Obstacle[0].Existance_Probability = 0.0f;
      fFreespace.a_Segment[ui].a_Obstacle[0].Emptiness_Probability = 1.0f;
    }

    fFanDef.f_fan_elem_width_rad = FRS_SEGMENT_WIDTH_RAD;
    fFanDef.f_fov_DEG = FRS_FAN_FOV_DEG;
    fFanDef.f_fov_rad = DEG2RAD(FRS_FAN_FOV_DEG);
    fFanDef.uiNumSegments = (uint32)FRS_NUM_SEGMENTS;
    fFanDef.Existance_Probability = FRS_CLUSTER_PROB_LOW;
    fFanDef.Emptiness_Probability = FRS_PROB_ONE - FRS_CLUSTER_PROB_LOW;

    frs_Points_to_Fan(&fFanDef, p_obstacles_FRS, &fFreespace);
    std::cout << "Freespace (seg) " << float(clock() - begin_time) / CLOCKS_PER_SEC << " [s]. " << std::endl;
  }
  // ===== BMW freespace =====
#endif

#ifdef COLOR_POINTS_PER_BIN
  draw points of each bin in another color (to verify correct) bin /raster drawing
  p_cloud_rgb->clear();
  for (int iBin = 0; iBin < N_POINTCLOUD_BINS; iBin++)
  {
    int r = (iBin * 30) % 255;
    int g = (75 + iBin * 30) % 255;
    int b = (150 + iBin * 30) % 255;
    for (int iPt = 0; iPt < v_bins[iBin].cloud->size(); iPt++)
    {
      pcl::PointXYZRGB prgb(r, g, b);
      prgb.x = v_bins[iBin].cloud->points[iPt].x;
      prgb.y = v_bins[iBin].cloud->points[iPt].y;
      prgb.z = v_bins[iBin].cloud->points[iPt].z;
      p_cloud_rgb->push_back(prgb);
    }
  }
#endif
  return 0;
}

void ground_patches_neighbours()
{
  int nBinRange = (int)VELODYNE_MAX_RANGE;
  for (int iBin = N_POINTCLOUD_BINS_X; iBin < N_POINTCLOUD_BINS-N_POINTCLOUD_BINS_X-1; iBin++)
  {
    if (v_bins[iBin].ground.gp_type == gpt_None)
    {
      uint16 iBinLeft = iBin-1;
      uint16 iBinRight = iBin + 1;
      uint16 iBinUp = iBin - N_POINTCLOUD_BINS_X;
      uint16 iBinDown = iBin + N_POINTCLOUD_BINS_X;
      int iBF=0;
      if (v_bins[iBinLeft].ground.gp_type == gpt_RANSAC) iBF = iBF | 1;
      if (v_bins[iBinRight].ground.gp_type == gpt_RANSAC) iBF = iBF | 2;
      if (v_bins[iBinUp].ground.gp_type == gpt_RANSAC) iBF = iBF | 4;
      if (v_bins[iBinDown].ground.gp_type == gpt_RANSAC) iBF = iBF | 8;
      if (iBF > 0)
      {
        float fSum = 
          (iBF & 1) * v_bins[iBinLeft].ground.z_avg +
          ((iBF & 2) >> 1) * v_bins[iBinRight].ground.z_avg +
          ((iBF & 4) >> 2) * v_bins[iBinUp].ground.z_avg +
          ((iBF & 8) >> 3) * v_bins[iBinDown].ground.z_avg;
        int contribute = (iBF & 1) + ((iBF & 2) >> 1) + ((iBF & 4) >> 2) + ((iBF & 8) >> 3);
        v_bins[iBin].ground.gp_type = gpt_Extrapolated;
        v_bins[iBin].ground.z_avg = fSum / contribute;
        std::cout << iBin << ": " << v_bins[iBin].ground.z_avg << std::endl;
      }
    }
  }
}

void calc_ground_patches_P1P4()
{
  int nBinRange = (int)VELODYNE_MAX_RANGE;
  float z;

  for (int _y = -nBinRange; _y < nBinRange; _y += POINTCLOUD_BINWIDTH)
  {
    for (int _x = -nBinRange; _x < nBinRange; _x += POINTCLOUD_BINWIDTH)
    {
      uint16 iBin = xy_to_bin(_x, _y);
      if (v_bins[iBin].ground.gp_type == gpt_RANSAC)
      {
        float a = v_bins[iBin].ground.coeff[0];
        float b = v_bins[iBin].ground.coeff[1];
        float c = v_bins[iBin].ground.coeff[2];
        float d = v_bins[iBin].ground.coeff[3];

        z = -(a*_x + b*_y + d) / c;

        float groundsquare = POINTCLOUD_BINWIDTH - 1.0f; // 2*0.5 .. patch füllt nicht das ganze bin aus
        pcl::PointXYZ p1, p2;
        p1.x = _x + 0.5f; // 0.5 .. patch füllt nicht das ganze bin aus
        p1.y = _y + 0.5f; // 0.5 .. patch füllt nicht das ganze bin aus
        p1.z = z;

        p2.x = p1.x;
        p2.y = p1.y + groundsquare;
        p2.z = -(a*p2.x + b*p2.y + d) / c;
        pcl::PointXYZ p3, p4;
        p3.x = p1.x + groundsquare;
        p3.y = p1.y + groundsquare;
        p3.z = -(a*p3.x + b*p3.y + d) / c;
        p4.x = p1.x + groundsquare;
        p4.y = p1.y;
        p4.z = -(a*p4.x + b*p4.y + d) / c;

        v_bins[iBin].ground.p1 = p1;
        v_bins[iBin].ground.p2 = p2;
        v_bins[iBin].ground.p3 = p3;
        v_bins[iBin].ground.p4 = p4;
        v_bins[iBin].ground.z_avg = (p1.z + p2.z + p3.z + p4.z) / 4.0f;
      }
    }
  }
}

void calc_ground_patches_P1P4_Extr() // ...ugly!!!
{
  int nBinRange = (int)VELODYNE_MAX_RANGE;
  float z;

  for (int _y = -nBinRange; _y < nBinRange; _y += POINTCLOUD_BINWIDTH)
  {
    for (int _x = -nBinRange; _x < nBinRange; _x += POINTCLOUD_BINWIDTH)
    {
      uint16 iBin = xy_to_bin(_x, _y);
      if (v_bins[iBin].ground.gp_type == gpt_Extrapolated)
      {
        z = v_bins[iBin].ground.z_avg;

        float groundsquare = POINTCLOUD_BINWIDTH - 1.0f; // 2*0.5 .. patch füllt nicht das ganze bin aus
        pcl::PointXYZ p1, p2;
        p1.x = _x + 0.5f; // 0.5 .. patch füllt nicht das ganze bin aus
        p1.y = _y + 0.5f; // 0.5 .. patch füllt nicht das ganze bin aus
        p1.z = z;

        p2.x = p1.x;
        p2.y = p1.y + groundsquare;
        p2.z = z;
        pcl::PointXYZ p3, p4;
        p3.x = p1.x + groundsquare;
        p3.y = p1.y + groundsquare;
        p3.z = z;
        p4.x = p1.x + groundsquare;
        p4.y = p1.y;
        p4.z = z;

        v_bins[iBin].ground.p1 = p1;
        v_bins[iBin].ground.p2 = p2;
        v_bins[iBin].ground.p3 = p3;
        v_bins[iBin].ground.p4 = p4;
      }
    }
  }
}
#endif // __PAULA_PROCESS__
