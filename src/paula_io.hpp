#pragma once

#include "kitty_evaluate_object.hpp"

#include "Vec3f.hxx" // to be included after pcl/eigen
#include "CSV.hxx"
#include <vector>

#include <windows.h> // read_directory()
#include <iostream>
#include <fstream>
#include <sstream> // save

// https://stackoverflow.com/questions/55344174/c-close-a-open-file-read-with-mmap
// for mmap (this is linux only)
#ifdef _WIN32
// https ://stackoverflow.com/questions/4087282/is-there-a-memory-mapping-api-on-windows-platform-just-like-mmap-on-linux
#include "mmap.h"
#else
#include <sys/mman.h>-
#include <sys/stat.h>
#include <fcntl.h>
#endif
#include "lzf.h"

#include <time.h>

bool b_Filechanged = false;

std::vector<std::string> v_argv; // vector of .pcd (.bin) filenames, passed by argument

int iFile = 0;                   // current position in that v_argv vector of .pcd files

typedef std::vector<string> stringvec;
stringvec v_files;

enum fld_type { ftF=0, ftU=1 };
enum pcd_type { pcd_ascii = 0, pcd_binary = 1, pcd_binary_compressed = 2, pcd_unknown = 255 };

struct s_pcd
{
  int nfields;
  std::vector<std::string> fields;
  std::vector<fld_type> type;
  std::vector<int> count;
  int width;
  int height;
  int npoints;
  pcd_type pcdtype;
};

struct PointXYZ
{
  float x;
  float y;
  float z;
};
typedef std::vector<PointXYZ> PointCloud;

struct PointXYZI
{
  float x;
  float y;
  float z;
  float intensity;
};

PointCloud * p_cloud; // later move to paula.h
char* data_uncomp; // uncompressed point cloud (pcd) .. better on heap

float bintofloat(unsigned int x);
int load_pcd_compressed(const std::string &filename, PointCloud &cloud);

// ATM this only works for binary non-compressed, 2do. combine with compressed. needs to read the header first!
int loadPCDbin(const std::string &filename, PointCloud &cloud)
{
  cloud.clear();
  // https://stackoverflow.com/questions/19614581/reading-floating-numbers-from-bin-file-continuosly-and-outputting-in-console-win

  float f;
  std::ifstream fin(filename, std::ios::binary);
  int i = 0;
  PointXYZI bincoord;
  std::vector<PointXYZI> bincloud;
  while (fin.read(reinterpret_cast<char*>(&f), sizeof(float)))
  {
    if (i == 0) bincoord.x = f;
    if (i == 1) bincoord.y = f;
    if (i == 2) bincoord.z = f;
    if (i == 3) bincoord.intensity = f;
    i++;
    if (i == 4)
    {
      bincloud.push_back(bincoord);
      i = 0;
    }
  }
  fin.close();
  for (int i = 0; i < bincloud.size(); i++)
  {
    PointXYZ p;
    p.x = bincloud[i].x;
    p.y = bincloud[i].y;
    p.z = bincloud[i].z;
    cloud.push_back(p);
  }
  return 0;
}

// https://stackoverflow.com/questions/53849/how-do-i-tokenize-a-string-in-c
std::vector<std::string> split(const char *str, char c = ' ')
{
  std::vector<std::string> result;
  do
  {
    const char *begin = str;
    while (*str != c && *str)
      str++;
    result.push_back(std::string(begin, str));
  } while (0 != *str++);

  return result;
}

int load_pcd_compressed(const std::string &filename, const s_pcd &pcd, PointCloud &cloud) {
  size_t len;
  int res = map_file(filename.c_str(), len);
  if (res != 0) return res;
  std::cout << "file-len = " << len << std::endl;
  
  // (a) jump over header
  char *ptr = (char *)lpBasePtr;
  LONGLONG i = len;
  int bheader = 0;
  int lenheader = 0;
  while ((bheader < 2) && (i-- > 0)) {
    if (bheader == 0)
    {
      if (((*ptr) == 'D') && (*(ptr + 1) == 'A') && (*(ptr + 2) == 'T') && (*(ptr + 3) == 'A'))
      {
        // no check for next 0xa
        bheader = 1;
      }
    }
    else if (bheader == 1)
    {
      if ((*ptr) == '\n')
      {
        // no check for next 0xa
        bheader = 2;
      }
    }
    ptr++;
    lenheader++;
  }

  // (b) uncompress, from https://github.com/PointCloudLibrary/pcl/blob/master/io/src/pcd_io.cpp
  unsigned int compressed_size = 0, len_uncomp = 0;
  memcpy(&compressed_size, ptr, 4);
  memcpy(&len_uncomp, (ptr+4), 4);

  data_uncomp = (char*)malloc(len_uncomp);
  unsigned int tmp_size = lzfDecompress((ptr+8), compressed_size, data_uncomp, len_uncomp);
  if (tmp_size != len_uncomp)
  {
    std::cout << "error uncompressing. size mismatch" << std::endl;
  }
  
  // (c) read the data to point cloud
  const int nfields = 4; // Hack, 2do: read #fields
//  const int nfields = pcd.nfields;
  int lenfloat = sizeof(float);
  i = len_uncomp / lenfloat;
//  int npts = i / nfields; // hack, instead read and parse the header
  int npts = pcd.npoints; // hack(2)

  // Unpack the x...xy...yz...z to xyzxyz
  // .... hmm, what to do here?
  char* ptr_field[nfields];
  ptr_field[0] = data_uncomp + 0 * npts * lenfloat; // x
  ptr_field[1] = data_uncomp + 1 * npts * lenfloat; // y
  ptr_field[2] = data_uncomp + 2 * npts * lenfloat; // z
  ptr_field[3] = data_uncomp + 3 * npts * lenfloat; // i
  // and copy to point cloud
  PointXYZ p;
  int j = -1;
  while (j++ < npts-1) {
    memcpy(&p.x, ptr_field[0] + j * lenfloat, lenfloat); // https://stackoverflow.com/questions/3991478/building-a-32-bit-float-out-of-its-4-composite-bytes
    memcpy(&p.y, ptr_field[1] + j * lenfloat, lenfloat);
    memcpy(&p.z, ptr_field[2] + j * lenfloat, lenfloat);
    cloud.push_back(p);
  }

  // (d) close up
  free(data_uncomp);
  close_mapfile(); // only 1 can be opened simultaneously

  return 0;
}

int load_pcd_flexi(const std::string &filename, PointCloud &cloud)
{
  cloud.clear();

  float f;
  std::ifstream fs(filename, std::ios::binary);
  bool bheader = true;
  bool beof = false;
  std::string line;
  std::vector<std::string> v_st; // string tokens
  pcd_type ftype = pcd_unknown;
  
  s_pcd pcd;

  int i = 0;
  PointXYZ p;
  while ((!beof) && (!fs.eof()))
  {
    if (bheader)
    {
      std::getline(fs, line);
      // Ignore empty lines
      if (line.empty())
        continue;

      // Tokenize the line
      v_st = split(line.c_str(), ' ');
      if (v_st[0] == "DATA") {
        v_st[1].erase(v_st[1].find_last_not_of(" \n\r\t") + 1); // trim string
        if (v_st[1] == "ascii") ftype = pcd_ascii;
        if (v_st[1] == "binary") ftype = pcd_binary;
        if (v_st[1] == "binary_compressed") ftype = pcd_binary_compressed;
        bheader = false;
      }
      else if (v_st[0] == "FIELDS") {
        int nfields = v_st.size() - 1;
        pcd.nfields = nfields;
        std::cout << line << std::endl;
      }
      else if (v_st[0] == "POINTS") {
        pcd.npoints = stoi(v_st[1]);
        std::cout << line << std::endl;
      }
    }
    else // body, 2do: read dynamic #fields
    {
      if (ftype == pcd_ascii)
      {
        std::getline(fs, line);
        // Ignore empty lines
        if (line.empty())
          continue;
        v_st = split(line.c_str(), ' ');
        p.x = std::stof(v_st[0]);
        p.y = std::stof(v_st[1]);
        p.z = std::stof(v_st[2]);
        cloud.push_back(p);
        i++;
      }
      else if (ftype == pcd_binary)
      {
        // we do differently than pcd, who forst close the file and then open as io::raw
        fs.read(reinterpret_cast<char*>(&f), sizeof(float));
        if (i == 0) p.x = f;
        if (i == 1) p.y = f;
        if (i == 2) p.z = f;
        if (i == 3);// p.intensity = f;
        i++;
        if (i == 4)
        {
          cloud.push_back(p);
          i = 0;
        }

      }
      else if (ftype == pcd_binary_compressed) {
//        std::cout << "binary compressed not supported." << std::endl;
//        std::cout << "binary compressed still in alpha phase." << std::endl;
        std::cout << "binary compressed now in beta phase." << std::endl;
        // header hier einlesen
        // hier den file pointer merken
        fs.close();
        beof = true;
      }
      else {
      // error
        return -1;
      }
    }
  }
  if (ftype != pcd_binary_compressed)
    fs.close();
  else {
    // file already closed above
    load_pcd_compressed(filename, pcd, cloud);
  }

  return 0;
}

int loadPCDFile(const std::string &filename, PointCloud &cloud) // this can only read ASCII content files!, takes 0.8sec to load a 5MB .PCD (release build)
{
  FILE *f;
  char line[255];
  char tmp[55];
  char tmp2[55];
  char tmp3[55];
  float coord[4];
  bool bheader = true;

  cloud.clear();
  fopen_s(&f, filename.c_str(), "r");
  while (fgets(line, sizeof(line), f))
  {
    if (bheader) sscanf(line, "%s %s %s", tmp, tmp2, tmp3);

    if (!bheader)
    {
      sscanf(line, "%f %f %f", &coord[0], &coord[1], &coord[2]);
      PointXYZ p;
      p.x = coord[0];
      p.y = coord[1];
      p.z = coord[2];
      cloud.push_back(p);
    }

    if (bheader)
    {
      if (std::string(tmp) == "DATA")// && (std::string(tmp2) == "ascii")) // omitting the "ascii" seems considerably faster !?
        bheader = false;
      if (std::string(tmp2) == "ascii")
        ;
      if (std::string(tmp2) == "binary")
        ;
    }
  }
  fclose(f);
  return 0;
}

int loadPCDFile_fast(const std::string &filename, PointCloud &cloud) // this seems to be only fast, loading into char[]
{
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  if (in)
  {
    std::string contents;
    in.seekg(0, std::ios::end);
    contents.resize(in.tellg());
    in.seekg(0, std::ios::beg);
    in.read(&contents[0], contents.size());
    in.close();
    const char * testCharArray = contents.c_str();

    std::vector<std::string> lines; // 2do: pre-size this!
    std::string line;
    bool bheader = true;
//    float coord[4];
//    int other[3];
    for (int i = 0; i < contents.size(); i++)                           // THIS
    {                                                                   // IS
      line = line + contents[i];                                        // REALLY
      if ((i > 0) && (contents[i-1] == '\r') && (contents[i] == '\n'))  // SLOW!!
      {
        lines.push_back(line);

/*        if (!bheader)
        {
          sscanf(line.c_str(), "%f %f %f %f %d %d %d\r\n", &coord[0], &coord[1], &coord[2], &coord[3], &other[0], &other[1], &other[2]);
          PointXYZ p;
          p.x = coord[0];
          p.y = coord[1];
          p.z = coord[2];
          cloud.push_back(p);
        }
*/
        if (bheader)
          if ((line[0] == 'D') && (line[1] == 'A') && (line[2] == 'T') && (line[3] == 'A'))
            bheader = false;

        line = "";
      }
    }
    
    return 0;
  }
  throw(errno);
  return -1;
}

int loadCSV_from_VeloView(const std::string &filename, PointCloud &cloud)
{
  std::ifstream file(filename);

  for (CSVIterator loop(file); loop != CSVIterator(); ++loop)
  {
    PointXYZ p;
    p.x = ::atof((*loop)[0].c_str());
    p.y = ::atof((*loop)[1].c_str());
    p.z = ::atof((*loop)[2].c_str());
    cloud.push_back(p);
  }
  return 0;
}

bool isPointcloudFileExtension(const std::string &filename_only)
{
  std::string file_ext = PathFindExtension(filename_only.c_str());
  return (
    (file_ext.compare(".bin") == 0) ||
    (file_ext.compare(".pcd") == 0) ||
    (file_ext.compare(".csv") == 0)
    );
}

int loadPointCloud(const std::string &filename, PointCloud &cloud)
{
  s_pointcloudfile pf;

  clock_t begin_time = clock();
  begin_time = clock();

  pf.filepath = filename;
  std::string s = PathFindFileName(filename.c_str());
  pf.pathname = filename.substr(0, filename.find(s));
  pf.filename = s.substr(0, s.find_last_of("."));        // e.g. "veloout"
  pf.file_ext = PathFindExtension(filename.c_str());     // e.g. ".pcd"
  if (pf.file_ext.compare(".bin") == 0) pf.ft = ft_BIN;
  if (pf.file_ext.compare(".pcd") == 0) pf.ft = ft_PCD;
  if (pf.file_ext.compare(".csv") == 0) pf.ft = ft_CSV;  // Veloview output
//  pclfile = pf; // set global var!

  if (pf.ft == ft_BIN)
  {
    if (loadPCDbin(filename, cloud) == -1)
    {
//      PCL_ERROR("Couldn't read file .bin\n");
      system("pause");
      return (-1);
    }
  }
  if (pf.ft == ft_PCD)
  {
//    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *p_cloud) == -1)
// really slow atm    if (loadPCDFile_fast(filename, *p_cloud) == -1)
    if (load_pcd_flexi(filename, *p_cloud) == -1)
    {
//      PCL_ERROR("Couldn't read file .pcd\n");
      std::cout << "Couldn't read file .pcd\n";
//      system("pause");
      return (-1);
    }
  }
  if (pf.ft == ft_CSV)
  {
    if (loadCSV_from_VeloView(filename, cloud) == -1)
    {
//      PCL_ERROR("Couldn't read file .csv\n");
      system("pause");
      return (-1);
    }
  }
  std::cout << "Loaded " << cloud.size() << " data points from .pcd/.bin with the following fields: " << std::endl;
//  std::cout << "Load__ = " << float(clock() - begin_time) / CLOCKS_PER_SEC << " [s]" << std::endl;
  return 0;
}

int savePCLbin(const std::string &filename, PointCloud &cloud)
{
  std::ofstream fout(filename, std::ios::binary);
  for (int j = 0; j < cloud.size(); j++)
  {
    PointXYZI pi{255,255,255,255};
    pi.x = cloud[j].x;
    pi.y = cloud[j].y;
    pi.z = cloud[j].z;
    float f_p[4];
    f_p[0] = pi.x;
    f_p[1] = pi.y;
    f_p[2] = pi.z;
    f_p[3] = pi.intensity;
    fout.write(reinterpret_cast<char*>(&f_p), sizeof(f_p));
  }
  fout.close();
  std::cout << "Pointcloud written to " << filename << std::endl;
  return 0;
}

#if 0
int saveLabel(const std::string &filename) // store objects (start with cars) ín Kitty Label Format
{
  std::ofstream outfile;
  outfile.open(filename);

  /*
    example:                  Darknet:
    =======                   =======
    obj_class   Car     vs.   class: class of object, in integer starting from 0
    truncation  0.00
    occulation  2
    alpha      -1.55
    x1        548.00    vs.   x, y: relative position of the center point of the rectangle, which are two floats range in (0, 1)
    y1        171.33
    x2        572.40
    y2        194.42
    h           1.48    vs.   h: relative height of the cuboid, which is float range in (0, 1)
    w           1.56    vs.   w, l: relative size of the rectangle, which are also two floats range in (0, 1)
    l           3.62
    y          -2.72
    z           0.82    vs.   z: relative height of the center of the cuboid, which is float range in (0, 1)
    x          48.22
    rz         -1.62    vs.   rz: orientation, which range in (-pi/2, pi/2)
  */

  stringstream stream;
  // check!!:  ry --> rz
  //           t1 --> x, t2 --> y, t3 --> z
  for (int i = 0; i < v_BBoxes.size(); i++)
  {
    if (v_BBoxes[i].objtype > obj_unknown)
    {
      tGroundtruth lbl;
      switch (v_BBoxes[i].objtype)
      {
      case tree:       lbl.box.type = "__Tree"; break; // not supported yet
      case car:        lbl.box.type = "Car";  break;
      case truck:      lbl.box.type = "__Truck"; break;
      case guardrail:  lbl.box.type = "__Guardrail"; break;
      case pedestrian: lbl.box.type = "Pedestrian";  break;
      }
      lbl.t1 = v_BBoxes[i].mass_center(0);
      lbl.t2 = v_BBoxes[i].mass_center(1);
      lbl.t3 = v_BBoxes[i].mass_center(2);
      lbl.h = v_BBoxes[i].OBB.h; // AUTOSAR: w ... 2do: fix!!
      lbl.w = v_BBoxes[i].OBB.w; // AUTOSAR: h ... 2do: fix!!
      lbl.l = v_BBoxes[i].OBB.l;
      lbl.ry = 0.0f;
      stream << lbl.box.type << " " <<
        std::fixed << setprecision(2) <<
        lbl.truncation << " " << lbl.occlusion << " " << lbl.box.alpha << " " <<
        lbl.box.x1 << " " << lbl.box.y1 << " " << lbl.box.x2 << " " << lbl.box.y2 << " " <<
        lbl.h << " " << lbl.w << " " << lbl.l << " " <<
        lbl.t1 << " " << lbl.t2 << " " << lbl.t3 << " " << lbl.ry << "\n"; // https://stackoverflow.com/questions/5373766/adding-a-newline-to-file-in-c
    }
  }
  outfile.write(stream.str().c_str(), stream.str().length());
  outfile.close();

  std::cout << "Labels written to " << filename << std::endl;
  return 0;
}
#endif

// return angle of plane normal vN to vUp (if vN is facing upward) or vDown
float32 planeAngleRAD(Vec3f vN)
{
  // https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
//  Vec3f vN = Vec3f(plane.coeff[0], plane.coeff[1], plane.coeff[2]);
  Vec3f vUp;
  if (vN.z > 0.0) vUp = Vec3f(0, 0, 1); else vUp = Vec3f(0, 0, -1); // vDown
  float fdot = Dot(vN, vUp);
  float fLenN = Length(vN);
  float fLenUp = Length(vUp);
  float fAngleRAD = acos(fdot / sqrt(fLenN * fLenUp));
  //  float fAngle = fAngleRAD* 180.0f / M_PI;
  return fAngleRAD;
}

// http://www.martinbroadhurst.com/list-the-files-in-a-directory-in-c.html
void read_directory(const std::string& name, stringvec& v)
{
  std::string pattern(name);
  pattern.append("\\*");
  WIN32_FIND_DATA data;
  HANDLE hFind;
  if ((hFind = FindFirstFile(pattern.c_str(), &data)) != INVALID_HANDLE_VALUE) {
    do {
      v.push_back(data.cFileName);
    } while (FindNextFile(hFind, &data) != 0);
    FindClose(hFind);
  }
}
