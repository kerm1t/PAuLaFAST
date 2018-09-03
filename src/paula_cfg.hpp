#ifndef __PAULA_CFG__
#define __PAULA_CFG__

#define VELODYNE_MAX_RANGE         60.0f // [m]   120.0f
#define POINTCLOUD_BINWIDTH         4.0f // [m]   2.5f --> worse results for real-scene
#define VELODYNE_TYPICAL_RANGE       8*POINTCLOUD_BINWIDTH // [m] e.g. to speed up visualization
                                                           // (has to be multiple of BINWIDTH, otherwise raster-drawing is wrong)

#define N_POINTCLOUD_BINS_X      (int)VELODYNE_MAX_RANGE*2/POINTCLOUD_BINWIDTH
#define N_POINTCLOUD_BINS_Y      (int)VELODYNE_MAX_RANGE*2/POINTCLOUD_BINWIDTH
#define N_POINTCLOUD_BINS             N_POINTCLOUD_BINS_X*N_POINTCLOUD_BINS_Y // -120...120 [m] split up x,y plane into e.g. 5x5 m bins

//#define B_DO_FREESPACE     false // false | true
//#define B_DO_CLUSTERING    false  // false | true
//#define B_REM_OUTLIERS     true // nicht einfach, hier einen sinnvollen Schwellwert zu finden
//#define B_DRAWRASTER       true  // false | true
#define B_DRAWVEHICLE      true  // false | true
#define B_DRAW_BIN_INDICES true
//bool b_REM_OUTLIERS;

// ego vehicle dimensions (+supression of ego reflections)
//#define EGO_VEHICLE_XSIZE           4.7f // [m]
//#define EGO_VEHICLE_YSIZE           1.8f // [m]
//#define EGO_REFL_SUPRESS_X         10.0f // [m]   much larger than vehicle dimensions (~Velodyne inaccuracy?)
                                    // street scene / PD : 10.0f
                                    // tunnel 9.0f (other ego vehicle!?)
//#define EGO_REFL_SUPRESS_Y          4.2f // [m]
//#define EGO_REFL_SUPRESS_X         EGO_VEHICLE_XSIZE + 1.0f // [m]   much larger than vehicle dimensions (~Velodyne inaccuracy?)
//#define EGO_REFL_SUPRESS_Y         EGO_VEHICLE_XSIZE + 1.0f // [m]
//#define EGO_REFL_SUPRESS_X_SIDE    EGO_REFL_SUPRESS_X/2.0f
//#define EGO_REFL_SUPRESS_Y_SIDE    EGO_REFL_SUPRESS_Y/2.0f
float fEGO_REFL_SUPRESS_X_SIDE;
float fEGO_REFL_SUPRESS_Y_SIDE;

// RANSAC Grid
//#define RANSAC_DISTTHRESH           0.1f // [m]  0.1f  / 0.075f
#define GROUNDSURFACE_MINPOINTS    10
//#define GROUNDSURFACE_Z_NORMAL_MIN  0.95f

// opt1
//#define OUTLIERS_RADIUS             0.2f//0.1f
//#define OUTLIERS_NEIGHBOURS_MIN     2
// opt2 (ped)
//#define OUTLIERS_RADIUS             0.1f // --> bad for far away obj's
//#define OUTLIERS_NEIGHBOURS_MIN     4

// NN Clustering
//#define CLUSTER_TOLERANCE           0.5f
//#define CLUSTER_TOLERANCE           0.3f  // opt2 (ped) ... distance dependent!!! or registration + accumulation (densifying)
#define CLUSTER_MINCOUNT           10
#define CLUSTER_MAXCOUNT        25000

#define PPROC_GROUNDOBJ_TOL_Z       0.3f // object mass center   vs.   4th RANSAC-plane coeff "d" (ax+bx+cx+d)


#endif // __PAULA_CFG__
