/*! \file **********************************************************************

  COMPANY:              Continental

  MODULNAME:            frs_synthGen_config.h

  DESCRIPTION:          Simulation Configuration - ... and online parameters

                        USAGE:
                           - include this header
                           - create an object of the class
                           - use classes members as configuration parameters
                           - EVERYTIME program default values for your input when <cfg_obj>.available() == 0 !!!
                           - for cyclic update just call <cfg_obj>.update()

  AUTHOR:               Robin Amar
  CREATION DATE:        04.28.2016

  VERSION:              $Revision: 1.1 $

**************************************************************************** */

#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <windows.h>
#include <stdio.h>
#include <atlstr.h>
#include <io.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

#pragma warning (disable: 4244)

#define PAULA_CONFIG_VERSION  1  // version number : RULE!! increment for adding or removing parameters

#define PAULA_CONFIG_UPDATE_TIME_S 1.0F // <-- 2do: insert check into calling function (sim_swc_main_vis.cpp)

#define PAULA_CONFIG_FILENAME "paula.cfg"


typedef unsigned int uint;

class C_PAuLa_Config
{
private:
  CString   file_name;
  int       initialised;
  long long time_since_last_modified;

public:
  // CONFIGURATION DATA
  int cfg_vers;               // version number of config reader -> see define
  int cfg_file_vers;          // version number of config file

  int iBIN_FILTER;
  
  int write_OBST;             // output <filename>_OBST.pcd

  float EGO_VEHICLE_XSIZE;
  float EGO_VEHICLE_YSIZE;
  float EGO_SUPRESS_MARGIN_X;
  float EGO_SUPRESS_MARGIN_Y;

  float CLUSTER_TOLERANCE;

  // [RANSAC_Grid]
  float RANSAC_DISTTHRESH;
  float GROUNDSURFACE_Z_NORMAL_MIN;
  
  // [Outliers]
  float OUTLIERS_RADIUS;
  float OUTLIERS_NEIGHBOURS_MIN;

  // [Do]
  int b_REM_OUTLIERS;
  int b_DO_FREESPACE;
  int b_DO_CLUSTERING;

  // [Visu]
  int b_DRAWRASTER;
  int b_DRAW_GROUND_PATCHES;
  int b_DRAW_GROUND_HEIGHT;
  int b_DRAW_GROUND_NORMAL;
  int b_DRAW_BIN_INDICES;
  int b_DRAW_OBJ_UNKNOWN;

/*
  struct s_Config_Paula
  {
    int ui_testCaseID;   // skip moving vehicle, bridges and in some recordings immobiles, too :-(
  };
  s_Config_Paula paulacfg;
*/

  C_PAuLa_Config()
  {
    cfg_vers = PAULA_CONFIG_VERSION;

    char buffer[MAX_PATH];
    GetModuleFileName((HINSTANCE)&__ImageBase, buffer, sizeof(buffer));
    std::string filepath = buffer;

    std::string s = PathFindFileName(filepath.c_str());
    std::string pathname = filepath.substr(0, filepath.find(s));
    std::string filename = s.substr(0, s.find_last_of("."));
    std::stringstream stream;
    stream << pathname << "..\\" << filename << ".cfg"; // .cfg in parent folder to be common for release an debug

    initialised = _access(stream.str().c_str(), 4) == 0;
    if (initialised) file_name = stream.str().c_str();
    else
    {
      HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
      SetConsoleTextAttribute(hStdout, FOREGROUND_RED | FOREGROUND_INTENSITY); // https://www.codeproject.com/Articles/16431/Add-color-to-your-std-cout
      std::cout << "cannot read config-file" << std::endl;
    }
    update();
  }

  ~C_PAuLa_Config()
  {
  }

  // -----------------------------------------
  // will crash, if string is not given in CFG
  // -----------------------------------------
  int read_cfg()
  {
    cfg_file_vers = GetCfgInt("GLOBAL", "Version", 999);

    // FRS
//    FRScfg.ui_testCaseID = GetCfgInt("FRS", "testCaseID", 0);

//    return FRScfg.ui_testCaseID;
    iBIN_FILTER = GetCfgInt("GLOBAL", "iBIN_FILTER", 0);

    write_OBST = GetCfgInt("GLOBAL", "write_OBST", 0);
    EGO_VEHICLE_XSIZE = GetCfgFloat("ego_vehicle_dimensions", "EGO_VEHICLE_XSIZE", 4.7);
    EGO_VEHICLE_YSIZE = GetCfgFloat("ego_vehicle_dimensions", "EGO_VEHICLE_YSIZE", 1.8);
    EGO_SUPRESS_MARGIN_X = GetCfgFloat("ego_vehicle_dimensions", "EGO_SUPRESS_MARGIN_X", 1.0);
    EGO_SUPRESS_MARGIN_Y = GetCfgFloat("ego_vehicle_dimensions", "EGO_SUPRESS_MARGIN_Y", 1.0);
 
    RANSAC_DISTTHRESH = GetCfgFloat("RANSAC_Grid", "RANSAC_DISTTHRESH", 0.1);
    GROUNDSURFACE_Z_NORMAL_MIN = GetCfgFloat("RANSAC_Grid", "GROUNDSURFACE_Z_NORMAL_MIN", 0.95);

    OUTLIERS_RADIUS = GetCfgFloat("Outliers", "OUTLIERS_RADIUS", 0.2);
    OUTLIERS_NEIGHBOURS_MIN = GetCfgInt("Outliers", "OUTLIERS_NEIGHBOURS_MIN", 2);

    b_REM_OUTLIERS = GetCfgInt("Processing_Steps", "b_REM_OUTLIERS", 0);
    b_DO_CLUSTERING = GetCfgInt("Processing_Steps", "b_DO_CLUSTERING", 0);
    b_DO_FREESPACE = GetCfgInt("Processing_Steps", "b_DO_FREESPACE", 0);

    b_DRAWRASTER = GetCfgInt("Draw", "b_DRAWRASTER", 0);
    b_DRAW_GROUND_PATCHES = GetCfgInt("Draw", "b_DRAW_GROUND_PATCHES", 0);
    b_DRAW_GROUND_HEIGHT = GetCfgInt("Draw", "b_DRAW_GROUND_HEIGHT", 0);
    b_DRAW_GROUND_NORMAL = GetCfgInt("Draw", "b_DRAW_GROUND_NORMAL", 0);
    b_DRAW_BIN_INDICES = GetCfgInt("Draw", "b_DRAW_BIN_INDICES", 0);
    b_DRAW_OBJ_UNKNOWN = GetCfgFloat("Draw", "b_DRAW_OBJ_UNKNOWN", 1);
    CLUSTER_TOLERANCE = GetCfgFloat("NN_Clustering", "CLUSTER_TOLERANCE", 0.3);
    return 0;
  }

  bool available()
  {
    return (initialised == 1);
  }

  int GetCfgInt(__in LPCSTR   lpSectName, __in LPCSTR lpKeyName, __in INT nDefault)
  {
    return GetPrivateProfileInt(lpSectName, lpKeyName, nDefault, file_name);
  }

  CString GetCfgStr(__in LPCSTR   lpSectName, __in LPCSTR lpKeyName, __in LPCSTR lpDefault)
  {
    char buffer[MAX_PATH];
    GetPrivateProfileStringA(lpSectName, lpKeyName, lpDefault, buffer, MAX_PATH, file_name);
    return CString(buffer);
  }

  float GetCfgFloat(__in LPCSTR   lpSectName, __in LPCSTR lpKeyName, __in INT nDefault)
  {
    char buffer[MAX_PATH];
    GetPrivateProfileString(lpSectName, lpKeyName, (LPCSTR)nDefault, buffer, MAX_PATH, file_name);
    return atof(buffer);
  }

  int reload()
  {
    C_PAuLa_Config();
    return initialised;
  }

  long long GetLastModifiedTime()
  {
    struct _stat st;
    int rc = _stat(file_name, &st);
    if (rc == -1)
      return -1;
    return st.st_mtime;
  }

  /* update: cyclic read of cfg file every LANE_CONFIG_UPDATE_TIME_S seconds
   *
   * returns -1 when not initialised
   *          0 when time not reached
   *          1 when config read
   */
  int update()
  {
    if (initialised)
    {
      long long lModTime = GetLastModifiedTime();
      if (lModTime == -1)
      {
        initialised = 0;
        return -1;
      }
      if (lModTime == time_since_last_modified)
        return 0;
      read_cfg();
      time_since_last_modified = lModTime;
      return 1;
    }
    else
    {
      return -1;
    }
  }
};
