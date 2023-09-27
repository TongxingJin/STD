#include "include/STDesc.h"
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
// debug
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include<pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace boost;

struct FixSizeCloudDeque{
  FixSizeCloudDeque(int s){
    size = s;
  }
  void push_back(pcl::PointCloud<pcl::PointXYZI> cloud){
    if(clouds.size() == size){
      clouds.pop_front();
      // LOG(INFO) << "After pop: " << clouds.size();
    }
    clouds.push_back(cloud);
  }
  int size;
  std::deque<pcl::PointCloud<pcl::PointXYZI>> clouds;
};

int findPoseIndexUsingTime(std::vector<double> &time_list, double &time) {
  double time_inc = 10000000000;
  int min_index = -1;
  for (size_t i = 0; i < time_list.size(); i++) {
    if (fabs(time_list[i] - time) < time_inc) {
      time_inc = fabs(time_list[i] - time);
      min_index = i;
    }
  }
  if (time_inc > 0.5) {
    std::string msg = "The timestamp between poses and point cloud is:" +
                      std::to_string(time_inc) + "s. Please check it!";
    ROS_ERROR_STREAM(msg.c_str());
    std::cout << "Timestamp for point cloud:" << time << std::endl;
    std::cout << "Timestamp for pose:" << time_list[min_index] << std::endl;
    exit(-1);
  }
  return min_index;
}

// std::vector<std::string> split_line(const std::string& s, const std::string& delimiter){
//   int left = 0;
//   int right = 0;
//   std::vector<std::string> result;
//   while((right = s.find(delimiter, left)) != std::string::npos){
//     if(left != right){
//       result.emplace_back(s.substr(left, right - left));
//     }
//     left = right + delimiter.size();
//   }
//   if(left != s.size()){
//     result.emplace_back(s.substr(left));
//   }
//   return result;
// }

void load_custom_pose_with_time(const std::string &pose_dir,
  std::vector<std::string> &tag_vec,
  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &poses_vec,
  std::vector<double> &times_vec){
  LOG(INFO) << "Dir: " << pose_dir;
  filesystem::path file_dir(pose_dir);
  filesystem::directory_iterator ite(file_dir);
  filesystem::directory_iterator non_ite;
  while(ite != non_ite){
    if(!filesystem::is_directory(*ite) && ite->path().extension() == ".txt"){
      tag_vec.emplace_back(ite->path().stem().string());
    }
    ite++;
  }
  sort(tag_vec.begin(), tag_vec.end());
  // for(const auto& s : tag_vector){
  //   LOG(INFO) << s;
  // }
  LOG(INFO) << "Tag vector size: " << tag_vec.size();
  for(int index = 0; index < tag_vec.size(); ++index){
    std::string tag = tag_vec[index];
    std::replace(tag.begin(), tag.end(), '_', '.');
    times_vec.emplace_back(std::stod(tag));
    // LOG(INFO) << std::setprecision(19) << times_vec.back();
    std::ifstream pose_file(pose_dir + "/" + tag_vec[index] + ".txt", std::ios::in);

    std::vector<double> poses;
    std::string line;
    int count = 0;
    Eigen::Vector3d position;
    Eigen::Matrix3d qua;
    while(getline(pose_file, line)){
      // std::istringstream ll(line);
      // std::string word;
      // while(getline(ll, word, ' '))
      // LOG(INFO) << "Line: " << line;
      std::vector<std::string> words = split_line(line, " ");
      // for(const auto& t : words){
      //   LOG(INFO) << t << "*";
      // }
      // assert(words.size() == 4);
      if(words.size() != 4){
        LOG(FATAL) << "Not 4: " << words.size();
      }
      for(const auto& value : words){
        poses.emplace_back(std::stod(value));
      }
      count++;
      if(count == 4){
        if(poses.size() != 16){
          LOG(FATAL) << "Data format is wrong!";
        }
        position << poses[3], poses[7], poses[11];
        qua << poses[0], poses[1], poses[2],
              poses[4], poses[5], poses[6],
              poses[8], poses[9], poses[10];//! 确认下行列主序
        poses_vec.emplace_back(std::make_pair(position, qua));//! 确认下内存对齐      
      }
    }
  }
  LOG(INFO) << "Tag size: " << tag_vec.size();
  LOG(INFO) << "Poses size: " << poses_vec.size();
  LOG(INFO) << "Time stamp size: " << times_vec.size();
  // assert(tag_vec.size() == poses_vec.size() == times_vec.size());
  
  pcl::PointCloud<pcl::PointXYZI> map;
  // pcl::PointCloud<pcl::PointXYZI> track;
  for(int index = 0; index < tag_vec.size(); ++index){
    pcl::PointCloud<pcl::PointXYZI> scan;
    pcl::io::loadPCDFile(pose_dir + "/" + tag_vec[index] + ".pcd", scan);
    down_sampling_voxel(scan, 0.2);
    Eigen::Matrix4d pose;
    pose << poses_vec[index].second, poses_vec[index].first,
            0, 0, 0, 1;
    pcl::transformPointCloud(scan, scan, pose);
    map += scan;
    pcl::PointXYZI p;
    p.x = poses_vec[index].first.x();
    p.y = poses_vec[index].first.y();
    p.z = poses_vec[index].first.z();
    // track.push_back(p);
    LOG(INFO) << "Index: " << index + 1 << "/" << tag_vec.size();
  }
  LOG(INFO) << "Map size: " << map.size();
  pcl::io::savePCDFileBinaryCompressed(pose_dir + "/map.pcd", map);
  // pcl::io::savePCDFileBinaryCompressed(pose_dir + "/track.pcd", track);
  // LOG(FATAL) << "Done";
  // for(int index = 1; index < tag_vec.size(); ++index){
  //   LOG(INFO) << "diff " << index << ": " << (poses_vec[index].first - poses_vec[index - 1].first).norm();
  // }

}

double GetYaw(const Eigen::Matrix3d& r) {
  double ret = 0.0;
  if (r(2, 0) < 1) {
    if (r(2, 0) > -1) {
      ret = std::atan2(r(1, 0), r(0, 0));
    } else {
      ret = -std::atan2(-r(1, 2), r(1, 1));
    }
  } else {
    ret = std::atan2(-r(1, 2), r(1, 1));
  }
  return ret;
}

Eigen::Matrix3d GetHorizon(const Eigen::Matrix3d& r) {
  Eigen::Vector3d ypr;

  if (r(2, 0) < 1) {
    if (r(2, 0) > -1) {
      ypr.y() = std::asin(-r(2, 0));
      ypr.x() = std::atan2(r(1, 0), r(0, 0));
      ypr.z() = std::atan2(r(2, 1), r(2, 2));
    } else {
      ypr.y() = M_PI_2;
      ypr.x() = -std::atan2(-r(1, 2), r(1, 1));
      ypr.z() = 0.0;
    }
  } else {
    ypr.y() = -M_PI_2;
    ypr.x() = std::atan2(-r(1, 2), r(1, 1));
    ypr.z() = 0.0;
  }

  return (Eigen::AngleAxisd((ypr.x()), Eigen::Vector3d::UnitZ())).toRotationMatrix();

  // return ypr;
}

STDescManager *std_manager;
pcl::PointCloud<pcl::PointXYZI>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_map(new pcl::PointCloud<pcl::PointXYZI>());
sensor_msgs::PointCloud2 map_msg;
pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>());
double current_time;// todo update current time
ros::Publisher pubAlignedCloud;
ros::Publisher pubGlobalMap;
pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
double fitness_score = 0.0;

// void ParseCloud(const livox_ros_driver::CustomMsg::ConstPtr &msg, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
//   current_cloud->clear();
//   int points_size = msg->point_num;
//   current_cloud->resize(points_size);
//   for(uint i=1; i<points_size; i++){
//     if(((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)){
//       cloud[i].x = msg->points[i].x;
//       cloud[i].y = msg->points[i].y;
//       cloud[i].z = msg->points[i].z;
//       cloud[i].intensity = msg->points[i].reflectivity;
//     }
//   }
//   return;
// }

// void PointCloudRelocCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg){
void PointCloudRelocCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    LOG(INFO) << "Callback";
    // ParseCloud(msg, current_cloud);
    pcl::fromROSMsg(*msg, *current_cloud);

    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> transformed_clouds;
    std::vector<double> scores;
    std::vector<int> candidate_ids;
    std::vector<int> rots;
    std::vector<Eigen::Matrix4d> delta_poses;
    for(int rot = 0; rot < 180; rot += 60){
      pcl::PointCloud<pcl::PointXYZI>::Ptr rot_cloud(new pcl::PointCloud<pcl::PointXYZI>);

      pcl::transformPointCloud<pcl::PointXYZI>(*current_cloud, *rot_cloud, Eigen::Vector3f::Zero(), Eigen::Quaternionf(Eigen::AngleAxisf(rot / 180.0 * M_PI, Eigen::Vector3f::UnitZ())));

      std::cout << "cloud size: " << rot_cloud->size() << std::endl;

      std::vector<STDesc> stds_vec;
      std_manager->GenerateSTDescs(rot_cloud, stds_vec);

      LOG(INFO) << "descriptor size: " << stds_vec.size();

      std::pair<int, double> search_result(-1, 0);
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
      loop_transform.first << 0, 0, 0;
      loop_transform.second = Eigen::Matrix3d::Identity();
      std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
      std_manager->SearchLoop(stds_vec, search_result, loop_transform,
                              loop_std_pair);
      if (search_result.first > 0) {
        std_manager->PlaneGeomrtricIcp(std_manager->plane_cloud_vec_.back(), std_manager->plane_cloud_vec_[search_result.first], loop_transform);
        Eigen::Matrix4d delta_pose;
        delta_pose << loop_transform.second, loop_transform.first,
                      0.0, 0.0, 0.0, 1.0;
        LOG(INFO) << "Delta: " << delta_pose;
        // pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        // pcl::transformPointCloud<pcl::PointXYZI>(*temp_cloud, *transformed_cloud, delta_pose);
        // transformed_clouds.push_back(transformed_cloud);
        // (retrieve_pose <- rot_align_pose)* (rot_align_pose <- align_pose) * (align_pose <- real_pose)
        // delta_pose * +rot * (from fastlio) //todo
        Eigen::Matrix4d offset;
        offset << Eigen::Quaterniond(Eigen::AngleAxisd(rot / 180.0 * M_PI, Eigen::Vector3d::UnitZ())).matrix(), 0.0,
                  0.0, 0.0, 0.0, 1.0;//todo check
        delta_poses.push_back(std_manager->poses_[search_result.first] * delta_pose * offset);
        scores.push_back(search_result.second);
        candidate_ids.push_back(search_result.first);
        rots.push_back(rot);
      }else{
        LOG(INFO) << "No loop is found!";
      }
      // auto t_query_end = std::chrono::high_resolution_clock::now();
      // querying_time.push_back(time_inc(t_query_end, t_query_begin));

      std_manager->ClearTmp();

      // temp_cloud->clear();//! 清除submap
      // keyCloudInd++;//! 关键帧序号自增
      // loop.sleep();
    }

    static int count = 0;
    if(count <= 5){
      map_msg.header.stamp = ros::Time::now();
      map_msg.header.frame_id = "camera_init";
      pubGlobalMap.publish(map_msg);
    }
    count++;
    
    if(!delta_poses.empty()){
      int best_id = std::distance(scores.begin(), std::max_element(scores.begin(), scores.end()));
      int best_frame_id = candidate_ids[best_id];
      LOG(INFO) << "***********************************************************[Loop Detection] triggle loop: "
                  << "--" << best_frame_id
                  << ", score:" << scores[best_id];

      // register verify
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      icp.setInputSource(current_cloud);
      icp.align(*transformed_cloud, delta_poses[best_id].cast<float>());
      if(icp.hasConverged()){
        double score = icp.getFitnessScore(1);
        LOG(INFO) << "ICP score: " << score;
        if(score < fitness_score){
          Eigen::Matrix4f final_trans = icp.getFinalTransformation();//todo
          sensor_msgs::PointCloud2 aligned_cloud_msg;
          pcl::toROSMsg(*transformed_cloud, aligned_cloud_msg);
          aligned_cloud_msg.header.stamp = msg->header.stamp;
          aligned_cloud_msg.header.frame_id = "camera_init";
          pubAlignedCloud.publish(aligned_cloud_msg); 
        }
      }


      // pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      // pcl::transformPointCloud<pcl::PointXYZI>(*current_cloud, *transformed_cloud, delta_poses[best_id]);
      
      
    }
    std::cout << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "demo_kitti");
  ros::NodeHandle nh;
  pubAlignedCloud = nh.advertise<sensor_msgs::PointCloud2>("/aligned_current", 100);
  pubGlobalMap = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 100);
  std::string dir_path = "";
  std::string bag_path = "";
  std::string pose_path = "";
  nh.param<std::string>("dir_path", dir_path, "");
  nh.param<std::string>("bag_path", bag_path, "");
  nh.param<std::string>("pose_path", pose_path, "");
  nh.param<double>("fitness_score_threshold", fitness_score, 0.0); 

  ConfigSetting config_setting;
  read_parameters(nh, config_setting);

  ros::Rate loop(500);
  ros::Rate slow_loop(10);
  pcl::io::loadPCDFile<pcl::PointXYZI>(dir_path + "map.pcd", *global_map);
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
  downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
  downSizeFilter.setInputCloud(global_map);
  downSizeFilter.filter(*global_map);
  pcl::toROSMsg(*global_map, map_msg);

  pcl::io::loadPCDFile<pcl::PointXYZI>(dir_path + "cropped_map.pcd", *cropped_map);
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter2;
  downSizeFilter2.setLeafSize(0.1, 0.1, 0.1);
  downSizeFilter2.setInputCloud(cropped_map);
  downSizeFilter2.filter(*cropped_map);

  icp.setMaxCorrespondenceDistance(0.1);
  icp.setTransformationEpsilon(1e-3);
  icp.setEuclideanFitnessEpsilon(1e-3);
  icp.setMaximumIterations(30);
  icp.setInputTarget(cropped_map);

  std_manager = new STDescManager(config_setting);
  filesystem::path file_dir(dir_path + "/../stds/");
  filesystem::directory_iterator ite(file_dir);
  filesystem::directory_iterator non_ite;
  std::vector<std::string> std_name_vec;
  while(ite != non_ite){
    if(!filesystem::is_directory(*ite) && ite->path().extension() == ".txt"){
      if(std::stoi(ite->path().stem().string()) <= 70){//todo
        std_name_vec.emplace_back(ite->path().stem().string());        
      }
    }
    ite++;
  }
  LOG(INFO) << std_name_vec.size() << " files are found!";
  sort(std_name_vec.begin(), std_name_vec.end(), [](std::string name1, std::string name2)-> bool {return std::stoi(name1) < std::stoi(name2);});
  //! todo
  std_manager->plane_cloud_vec_.resize(std::stoi(std_name_vec.back()) + 1);
  std_manager->current_frame_id_ = std::stoi(std_name_vec.back()) + 1;
  std_manager->std_nums_.resize(std::stoi(std_name_vec.back()) + 1, 0);
  std_manager->poses_.resize(std::stoi(std_name_vec.back()) + 1, Eigen::Matrix4d::Identity());
  for(const std::string& file_name : std_name_vec){
    LOG(INFO) << "Loading from " << dir_path << "/../stds/" << file_name;
    std_manager->LoadFromFile(dir_path + "/../stds/" + file_name);
  }
  LOG(INFO) << "Database size: " << std_manager->data_base_.size();
  LOG(INFO) << "Current frame id: " << std_manager->current_frame_id_;

  ros::Subscriber cloud_sub = nh.subscribe("/cloud_global_aligned", 1000, PointCloudRelocCallback);

  ros::spin();

  return 0;
}