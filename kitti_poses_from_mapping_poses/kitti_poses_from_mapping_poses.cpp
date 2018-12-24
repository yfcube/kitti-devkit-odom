#include <iostream>
#include <string>
#include <Eigen/Geometry>
#include <fstream>
#include <experimental/filesystem>
#include <sstream>

namespace fs = std::experimental::filesystem;

struct Data
{
  int seq;
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  Data():seq(0),x(0),y(0),z(0),roll(0),pitch(0),yaw(0){}
};

int main(int argc, char const *argv[])
{
  /* code */
  if (argc != 2)
  {
    std::cout << "invalid number of input arguments, expect 2 args." << std::endl;
    std::cout << "example: $./getKittiGT calib_dir data_dir start_index end_index" << std::endl;
    exit(-1);
  }

  fs::path data_path(argv[1]);
  std::string data_file(data_path.string());
  
  std::string output_file = data_path.parent_path().string() + "/mapping_poses_kitti_format.txt";
  
  std::ifstream in(data_file);
  if(!fs::exists(data_file))
  {
    std::cerr << "No such file: " << data_file << std::endl;
    exit(-1);
  }

  std::ofstream os;
  os.open(output_file, std::ios::out|std::ios::trunc);

  int first_frame = -1;
  int last_frame = -1;
  int count = 0;
  std::string line;
  while(std::getline(in, line))
  {
    std::istringstream line_stream(line);
    Data data;
    line_stream >> data.seq >> data.x >> data.y >> data.z >> data.roll >> data.pitch >> data.yaw;

    if(first_frame<0)
      first_frame = data.seq;
    last_frame = data.seq;
    
    Eigen::AngleAxisd roll(data.roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(data.pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(data.yaw, Eigen::Vector3d::UnitZ());
    auto rotation = yaw*pitch*roll;
    Eigen::Translation3d trans(data.x, data.y, data.z);
    
    // std::cout << data.seq<< " " <<  data.x <<" "<< data.y <<" "<< data.z <<" "<< data.roll <<" "<<data.pitch<<" "<<data.yaw<< std::endl;
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> pose = (trans*rotation).matrix();
    for(int i=0; i<12; i++)
    {
      os<< *(pose.data()+i) << " ";
    }
    os<<"\n";
    count++;
  }

  os.close();
  std::cout << "Finished " << count << " frames: " <<" from " << first_frame << " to " << last_frame <<std::endl;
  std::cout << "Stored in: " << output_file << std::endl;
  return 0;
}
