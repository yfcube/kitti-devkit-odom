#include <iostream>
#include <fstream>
#include <experimental/filesystem>
#include <string>
#include <Eigen/Geometry>

namespace fs = std::experimental::filesystem;
double scale = -1;
bool first = true;

enum DataIndex
{
  LATITUDE = 0,
  LONGITUDE,
  ALTITUDE,
  ROLL,
  PITCH,
  YAW,
  VEL_NORTH,
  VEL_EAST,
  VEL_UP,
  VEL_F,
  VEL_L,
  VEL_U,
  ACC_X,
  ACC_Y,
  ACC_Z,
  ANG_VEL_X,
  ANG_VEL_Y,
  ANG_VEL_Z,
  ANG_VEL_F,
  ANG_VEL_L,
  ANG_VEL_U,
  POS_ACCURACY,
  VEL_ACCURACY,
  NAVSTAT,
  NUMSTAT,
  POSMODE,
  VELMODE,
  ORIMODE
};

double latToScale(const double &latitute)
{
  return cosf(latitute / M_PI * 180.0);
}

Eigen::Vector2d latlonToMercator(double lat, double lon, double scale)
{
  static double ER = 6378137;
  double x = scale * lon * M_PI * ER / 180.0;
  double y = scale * ER * log(tan((90 + lat) * M_PI / 360.0));

  return Eigen::Vector2d(x, y);
}

// Reference: Andreas Geiget, Vision meets Robotics: The KITTI Dataset
Eigen::Matrix4Xd poseFromLatLonAlt(const double &lat, const double &lon, const double &alt,
                                   const double &roll, const double &pitch, const double &yaw)
{
  static bool initialized = false;
  static Eigen::Matrix4d T_0_invserse;

  if (scale < 0)
  {
    std::cerr << "no scale" << std::endl;
    exit(-1);
  }

  Eigen::AngleAxisd r(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd p(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd y(roll, Eigen::Vector3d::UnitZ());
  auto rotation = y * p * r;
  auto xy = latlonToMercator(lat, lon, scale);
  auto translation = Eigen::Translation3d(xy[0], xy[1], alt);

  if (!initialized)
  {
    T_0_invserse = (translation * rotation).matrix().inverse();
    initialized = true;
    return Eigen::Matrix4d::Identity();
  }

  return T_0_invserse * (translation * rotation).matrix();
}

int getNumLength(const int &num)
{
  int base = 10;
  int i = 0;
  bool done = false;
  while (!done)
  {
    if (num == num % base)
      done = true;
    base = base * 10;
    i++;
  }
  return i;
}

std::string getPad(const int &num)
{
  int count = num;
  std::string pad;
  if (num < 0)
  {
    std::cerr << "invalid argument to getPad: " << num << std::endl;
    exit(-1);
  }

  while (count > 0)
  {
    pad = pad + "0";
    count--;
  }
  return pad;
}

int main(int argc, char const *argv[])
{
  /* code */
  if (argc != 5)
  {
    std::cout << "invalid number of input arguments, expect 5 args." << std::endl;
    std::cout << "example: $./getKittiGT calib_dir data_dir start_index end_index" << std::endl;
    exit(-1);
  }

  int start = std::atoi(argv[3]);
  int end = std::atoi(argv[4]);

  // get calibration data
  std::string calibr_dir(argv[1]);
  std::string data_dir(argv[2]);
  
  std::string output_file = data_dir + "/gt_from_rtk.txt";
  data_dir = data_dir + "/oxts/data/";

  int num = start;
  // bool file_not_found = false;

  std::ofstream os;
  os.open(output_file, std::ios::out | std::ios::trunc);
  while (num<=end)
  {
    std::string data_file = data_dir + getPad(10 - getNumLength(num)) + std::to_string(num) + ".txt";
    std::cout << data_file << std::endl;

    if (!fs::exists(data_file))
    {
      std::cerr << "no such file: " << data_file << std::endl;
      file_not_found = true;
      continue;
    }

    std::ifstream in(data_file, std::ios::in);
    std::vector<double> data(30, 0);
    int index = 0;
    while (!in.eof())
    {
      in >> data[index++];
    }
    if (first)
    {
      first = false;
      scale = latToScale(data[LATITUDE]);
    }

    // specify a row-major storage matrix
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> pose =
        poseFromLatLonAlt(data[LATITUDE], data[LONGITUDE], data[ALTITUDE],
                          data[ROLL], data[PITCH], data[YAW]);


    for(int i=0; i<12;i++)
    {
      os << *(pose.data()+i) <<" " ;
    }
    os << "\n";

    // std::cout.precision(8);
    // // std::cout << std::fixed << data[LATITUDE] << "\t" << data[LONGITUDE] << "\t" << data[ALTITUDE] << std::endl;
    // std::cout << std::fixed << pose << std::endl;
    // std::cout << "\n" << std::endl;
    // for(int i=0; i<12;i++)
    // {
    //   std::cout << *(pose.data()+i) <<" " ;
    // }
    // std::cout << "\n----\n" << std::endl;

    num++;
  }
  os.close();
  std::cout << "Finished. gt file stored as: "<< output_file << std::endl;

  return 0;
}
