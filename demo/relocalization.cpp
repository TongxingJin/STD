#include "include/STDesc.h"
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
// debug
#include <pcl/io/pcd_io.h>
// #include <pcl/filters/voxel_grid.h>
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

// Eigen::Matrix3d YPRAngles2Rotation(double yaw, double pitch,
//                                               double roll) {
//   return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
//       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
//       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "demo_kitti");
  ros::NodeHandle nh;
  std::string dir_path = "";
  std::string bag_path = "";
  std::string pose_path = "";
  nh.param<std::string>("dir_path", dir_path, "");
  nh.param<std::string>("bag_path", bag_path, "");
  nh.param<std::string>("pose_path", pose_path, "");

  ConfigSetting config_setting;
  read_parameters(nh, config_setting);



  ros::Publisher pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  ros::Publisher pubCureentCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
  ros::Publisher pubCurrentCorner =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
  ros::Publisher pubMatchedCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
  ros::Publisher pubMatchedCorner =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
  ros::Publisher pubSTD =
      nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);

  ros::Rate loop(500);
  ros::Rate slow_loop(10);

  STDescManager *std_manager = new STDescManager(config_setting);
  filesystem::path file_dir(dir_path + "/../stds/");
  filesystem::directory_iterator ite(file_dir);
  filesystem::directory_iterator non_ite;
  std::vector<std::string> std_name_vec;
  while(ite != non_ite){
    if(!filesystem::is_directory(*ite) && ite->path().extension() == ".txt"){
      if(std::stoi(ite->path().stem().string()) <= 70){
        std_name_vec.emplace_back(ite->path().stem().string());        
      }
    }
    ite++;
  }
  LOG(INFO) << std_name_vec.size() << " files are found!";
  sort(std_name_vec.begin(), std_name_vec.end(), [](std::string name1, std::string name2)->bool{return std::stoi(name1) < std::stoi(name2);});
  //! todo
  std_manager->plane_cloud_vec_.resize(std::stoi(std_name_vec.back()) + 1);
  std_manager->current_frame_id_ = std::stoi(std_name_vec.back()) + 1;
  std_manager->std_nums_.resize(std::stoi(std_name_vec.back()) + 1, 0);
  for(const std::string& file_name : std_name_vec){
    // if(std::stoi(file_name) > 300 && std::stoi(file_name) < 750){
    //   continue;
    // }
    LOG(INFO) << "Loading from " << dir_path << "/../stds/" << file_name;
    std_manager->LoadFromFile(dir_path + "/../stds/" + file_name);
  }
  LOG(INFO) << "Database size: " << std_manager->data_base_.size();
  LOG(INFO) << "Current frame id: " << std_manager->current_frame_id_;

  // std::vector<std::string> tag_vec;
  // std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> poses_vec;
  // std::vector<double> times_vec;
  // load_custom_pose_with_time(dir_path, tag_vec, poses_vec, times_vec);//! 读取时间戳和位姿到vector
  // std::vector<int> pcd_index(times_vec.size());
  // for(int index = 0; index < times_vec.size(); ++index){
  //   pcd_index[index] = index;
  // }
  // std::cout << "Sucessfully load pose with number: " << poses_vec.size()
  //           << std::endl;

  // size_t cloudInd = 0;
  // size_t keyCloudInd = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());

  std::vector<double> descriptor_time;
  std::vector<double> querying_time;
  std::vector<double> update_time;
  // int triggle_loop_num = 0;

  // while (ros::ok()) {
    // int true_loop_frames = 0;
    // int true_positive_recall = 0;//! 正确回环上
    // int true_in_reality = 0;
    // int true_positive_precision = 0;
    // int true_loop_frames_tmp = 0;
    // int true_positive_recall_tmp = 0;//! 正确回环上
    // int true_in_reality_tmp = 0;
    // int true_positive_precision_tmp = 0;
    // pcl::PointCloud<pcl::PointXYZ> pose_tree;
    // pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    // FixSizeCloudDeque queue(10);



    // BOOST_FOREACH (int i, pcd_index) {//! view里的每个m。遍历，并非多线程！
    //     double laser_time = times_vec[i];
    //     pcl::PointCloud<pcl::PointXYZI> cloud;
    //     pcl::io::loadPCDFile(dir_path + "/" + tag_vec[i] + ".pcd", cloud);
    //     Eigen::Vector3d translation = poses_vec[i].first;
    //     Eigen::Matrix3d rotation = poses_vec[i].second;
        
    //     for (size_t i = 0; i < cloud.size(); i++) {
    //       Eigen::Vector3d pv = point2vec(cloud.points[i]);
    //       pv = rotation * pv + translation;
    //       cloud.points[i] = vec2point(pv);
    //     }//! 点云转到全局坐标下
    //     down_sampling_voxel(cloud, config_setting.ds_size_);
    //     // for (auto pv : cloud.points) {
    //     //   temp_cloud->points.push_back(pv);//! subumap
    //     // }
    //     queue.push_back(cloud);

    //     // check if keyframe
    //     if (cloudInd % config_setting.sub_frame_num_ == 0 && cloudInd != 0) {
    //       for(const auto& cloud : queue.clouds){
    //         *temp_cloud += cloud;
    //       }
        for(int pcd_id = 20; pcd_id <= 74; pcd_id += 1){
          // if(pcd_id < 750 && pcd_id > 300) continue;
          // LOG(INFO) << "Database Plan Num: " << std_manager->plane_cloud_vec_.size();
          // LOG(INFO) << "Target plane size: " << std_manager->plane_cloud_vec_[pcd_id]->size();
          for(int rot = 0; rot < 180; rot += 30){
          std::string reloc_dir_path = "/media/jin/MyPassport/NTU_HELMET/0917/2/2023-09-17-22-07-29/reloc/";
          pcl::io::loadPCDFile(reloc_dir_path + "/pcd/" + std::to_string(pcd_id) + ".pcd", *temp_cloud);
          pcl::transformPointCloud<pcl::PointXYZI>(*temp_cloud, *temp_cloud, Eigen::Vector3f::Zero(), Eigen::Quaternionf(Eigen::AngleAxisf(rot / 180.0 * M_PI, Eigen::Vector3f::UnitZ())));
          LOG(INFO) << "Current pcd id: " << pcd_id;
          // down_sampling_voxel(*temp_cloud, config_setting.ds_size_);
          // std::cout << "Key Frame id:" << keyCloudInd
          //           << ", cloud size: " << temp_cloud->size() << std::endl;
          std::cout << "cloud size: " << temp_cloud->size() << std::endl;
          // step1. Descriptor Extraction
          auto t_descriptor_begin = std::chrono::high_resolution_clock::now();
          std::vector<STDesc> stds_vec;
          std_manager->GenerateSTDescs(temp_cloud, stds_vec);//todo 带着绝对值的
          // LOG(INFO) << "Current frame plane num: " << std_manager->plane_cloud_vec_.back()->size();

          LOG(INFO) << "descriptor size: " << stds_vec.size();
          auto t_descriptor_end = std::chrono::high_resolution_clock::now();
          descriptor_time.push_back(
              time_inc(t_descriptor_end, t_descriptor_begin));
          // step2. Searching Loop
          auto t_query_begin = std::chrono::high_resolution_clock::now();
          std::pair<int, double> search_result(-1, 0);
          std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
          loop_transform.first << 0, 0, 0;
          loop_transform.second = Eigen::Matrix3d::Identity();
          std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
          // if (keyCloudInd > config_setting.skip_near_num_) {//! 最开始数据库低于50的时候不找？
            std_manager->SearchLoop(stds_vec, search_result, loop_transform,
                                    loop_std_pair);
          // }
          if (search_result.first > 0) {
            std::cout << "***********************************************************[Loop Detection] triggle loop: " << pcd_id
                      << "--" << search_result.first
                      << ", score:" << search_result.second << std::endl;
            // LOG(INFO) << loop_transform.second;
            std_manager->PlaneGeomrtricIcp(std_manager->plane_cloud_vec_.back(), std_manager->plane_cloud_vec_[search_result.first], loop_transform);
            // LOG(INFO) << loop_transform.second;
            filesystem::create_directory(reloc_dir_path + "/reloc/");
            Eigen::Matrix4d delta_pose;
            delta_pose << loop_transform.second, loop_transform.first,
                          0.0, 0.0, 0.0, 1.0;
            // LOG(INFO) << "Delta: " << delta_pose;
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud<pcl::PointXYZI>(*temp_cloud, *transformed_cloud, delta_pose);
            LOG(INFO) << "Saving into " << reloc_dir_path + "/reloc/" + std::to_string(pcd_id) + "_" + std::to_string(search_result.first) + ".pcd";
            pcl::io::savePCDFileBinaryCompressed(reloc_dir_path + "/reloc/" + std::to_string(pcd_id) + "_" + std::to_string(search_result.first) + ".pcd", *transformed_cloud);            
          }else{
            LOG(INFO) << "No loop is found!";
          }
          auto t_query_end = std::chrono::high_resolution_clock::now();
          querying_time.push_back(time_inc(t_query_end, t_query_begin));

          //!
          // LOG(INFO) << "Before clear: " << std_manager->plane_cloud_vec_.size();
          std_manager->ClearTmp();
          // LOG(INFO) << "After clear: " << std_manager->plane_cloud_vec_.size();
          //!

          //! debug
          {
            // std::vector<int> indexs;
            // std::vector<float> dis;
            // pcl::PointXYZ pose_point;
            // pose_point.x = translation.x();
            // pose_point.y = translation.y();
            // pose_point.z = translation.z();
            // if(keyCloudInd > config_setting.skip_near_num_){
            //   if(tree.radiusSearch(pose_point, 20, indexs, dis) > 0){
            //     for(int id : indexs){
            //       if (id < pose_tree.size() - config_setting.skip_near_num_){
            //         true_loop_frames++;
            //         if(search_result.first > 0 && (poses_vec[search_result.first].first - poses_vec[i].first).norm() < 20){
            //           true_positive_recall++;
            //         }
            //         break;
            //       }
            //     }
                
            //   }
            //   if(search_result.first > 0){
            //     true_in_reality++;
            //     if((poses_vec[search_result.first].first - poses_vec[i].first).norm() < 20){
            //       true_positive_precision++;
            //     }
            //   }
            // }
          }

          // bool relocalization = false;
          // if(relocalization){
          //   // start of relocalization, transform cloud back into current pose
          //   // step1. Descriptor Extraction
          //   std::cout << "**************relocalization" << std::endl;
          //   pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
          //   *tmp_cloud = *temp_cloud;
          //   // {// align to gravity
          //   //   double current_yaw = GetYaw(rotation);
          //   //   LOG(INFO) << "Yaw: " << current_yaw / M_PI * 180.0;
          //   //   Eigen::Matrix3d tmp_rot = Eigen::AngleAxisd(current_yaw, Eigen::Vector3d::UnitZ()).matrix();
              
          //   //   for (size_t i = 0; i < tmp_cloud->size(); i++) {
          //   //     Eigen::Vector3d pv = point2vec(tmp_cloud->points[i]);
          //   //     pv = tmp_rot.inverse() * (pv - translation);
          //   //     tmp_cloud->points[i] = vec2point(pv);
          //   //   }
          //   // }
          //   for (size_t i = 0; i < tmp_cloud->size(); i++) {
          //     Eigen::Vector3d pv = point2vec(tmp_cloud->points[i]);
          //     pv = rotation.inverse() * (pv - translation);
          //     tmp_cloud->points[i] = vec2point(pv);
          //   }
          //   std::vector<STDesc> stds_vec;
          //   std_manager->GenerateSTDescs(tmp_cloud, stds_vec);
          //   LOG(INFO) << "descriptor size: " << stds_vec.size();
          //   std_manager->ClearTmp();// Recover the change to member variables caused by GenerateSTDescs

          //   // step2. Searching Loop
          //   std::pair<int, double> search_result(-1, 0);
          //   std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
          //   loop_transform.first << 0, 0, 0;
          //   loop_transform.second = Eigen::Matrix3d::Identity();
          //   std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
          //   if (keyCloudInd > config_setting.skip_near_num_) {
          //     std_manager->SearchLoop(stds_vec, search_result, loop_transform,
          //                             loop_std_pair);
          //   }
          //   if (search_result.first > 0) {
          //     std::cout << "[Loop Detection] triggle loop: " << keyCloudInd
          //               << "--" << search_result.first
          //               << ", score:" << search_result.second << std::endl;
          //   }else{
          //     LOG(INFO) << "No loop is found!";
          //   }

          //   //! debug
          //   {
          //     std::vector<int> indexs;
          //     std::vector<float> dis;
          //     pcl::PointXYZ pose_point;
          //     pose_point.x = translation.x();
          //     pose_point.y = translation.y();
          //     pose_point.z = translation.z();
          //     if(keyCloudInd > config_setting.skip_near_num_){
          //       if(tree.radiusSearch(pose_point, 20, indexs, dis) > 0){
          //         for(int id : indexs){
          //           if (id < pose_tree.size() - config_setting.skip_near_num_){
          //             true_loop_frames_tmp++;
          //             if(search_result.first > 0 && (poses_vec[search_result.first].first - poses_vec[i].first).norm() < 20){
          //               true_positive_recall_tmp++;
          //             }
          //             break;
          //           }
          //         }
          //       }
          //       if(search_result.first > 0){
          //         true_in_reality_tmp++;
          //         if((poses_vec[search_result.first].first - poses_vec[i].first).norm() < 20){
          //           true_positive_precision_tmp++;
          //         }
          //       }
          //     }
          //   }
          //   std::cout << "end of relocalization**************" << std::endl;
          // }// end of relocalization

          // // step3. Add descriptors to the database
          // auto t_map_update_begin = std::chrono::high_resolution_clock::now();
          // // todo: save std desciptors
          // std::ofstream of(dir_path + "/../stds/" + std::to_string(stds_vec.front().frame_id_) + ".txt", std::ios::out);
          // for (auto single_std : stds_vec) {
          //   of << single_std.side_length_(0) << " " << single_std.side_length_(1) << " " << single_std.side_length_(2) << " " <<
          //         single_std.angle_(0) << " " << single_std.angle_(1) << " " << single_std.angle_(2) << " " <<
          //         single_std.center_(0) << " " << single_std.center_(1) << " " << single_std.center_(2) << " " << 
          //         single_std.frame_id_ << " " <<
          //         single_std.vertex_A_(0) << " " << single_std.vertex_A_(1) << " " << single_std.vertex_A_(2) << " " <<
          //         single_std.vertex_B_(0) << " " << single_std.vertex_B_(1) << " " << single_std.vertex_B_(2) << " " <<
          //         single_std.vertex_C_(0) << " " << single_std.vertex_C_(1) << " " << single_std.vertex_C_(2) << " " <<
          //         single_std.vertex_attached_(0) << " " << single_std.vertex_attached_(1) << " " << single_std.vertex_attached_(2) << "\n";
          // }
          // of.close();
          // std_manager->AddSTDescs(stds_vec);
          // auto t_map_update_end = std::chrono::high_resolution_clock::now();
          // update_time.push_back(time_inc(t_map_update_end, t_map_update_begin));
          // std::cout << "[Time] descriptor extraction: "
          //           << time_inc(t_descriptor_end, t_descriptor_begin) << "ms, "
          //           << "query: " << time_inc(t_query_end, t_query_begin)
          //           << "ms, "
          //           << "update map:"
          //           << time_inc(t_map_update_end, t_map_update_begin) << "ms"
          //           << std::endl;

          // pcl::PointCloud<pcl::PointXYZI> save_key_cloud;
          // save_key_cloud = *temp_cloud;

          // std_manager->key_cloud_vec_.push_back(save_key_cloud.makeShared());


          // LOG(INFO) << "Before publish";
          // publish

          // sensor_msgs::PointCloud2 pub_cloud;
          // pcl::toROSMsg(*temp_cloud, pub_cloud);
          // pub_cloud.header.frame_id = "camera_init";
          // pub_cloud.header.stamp = ros::Time::now();
          // pubCureentCloud.publish(pub_cloud);//! 发布当前submap
          // pcl::toROSMsg(*std_manager->corner_cloud_vec_.back(), pub_cloud);
          // pub_cloud.header.frame_id = "camera_init";
          // pub_cloud.header.stamp = ros::Time::now();
          // pubCurrentCorner.publish(pub_cloud);//! 发布当前submap的特征点

          // if (search_result.first > 0) {
          //   triggle_loop_num++;
          //   pcl::toROSMsg(*std_manager->key_cloud_vec_[search_result.first],
          //                 pub_cloud);
          //   pub_cloud.header.frame_id = "camera_init";
          //   pub_cloud.header.stamp = ros::Time::now();
          //   pubMatchedCloud.publish(pub_cloud);
          //   slow_loop.sleep();//! 发布有一定的时间滞后
          //   pcl::toROSMsg(*std_manager->corner_cloud_vec_[search_result.first],
          //                 pub_cloud);
          //   pub_cloud.header.frame_id = "camera_init";
          //   pub_cloud.header.stamp = ros::Time::now();
          //   pubMatchedCorner.publish(pub_cloud);
          //   // publish_std_pairs(loop_std_pair, pubSTD);
          //   // slow_loop.sleep();
          //   // getchar();
          // }else{
            // sensor_msgs::PointCloud2 empty_cloud;
            // empty_cloud.header.frame_id = "camera_init";
            // empty_cloud.header.stamp = ros::Time::now();
            // pubMatchedCloud.publish(empty_cloud);
            // pubMatchedCorner.publish(empty_cloud);
          // }
          // publish_std_pairs(loop_std_pair, pubSTD);//! 清除旧的
          // slow_loop.sleep();


        // }
          // nav_msgs::Odometry odom;
          // odom.header.frame_id = "camera_init";
          // odom.header.stamp = ros::Time::now();
          // odom.pose.pose.position.x = translation[0];
          // odom.pose.pose.position.y = translation[1];
          // odom.pose.pose.position.z = translation[2];
          // Eigen::Quaterniond q(rotation);
          // odom.pose.pose.orientation.w = q.w();
          // odom.pose.pose.orientation.x = q.x();
          // odom.pose.pose.orientation.y = q.y();
          // odom.pose.pose.orientation.z = q.z();
          // pubOdomAftMapped.publish(odom);//! 所有帧都发布odo
          // loop.sleep();

          // pcl::PointXYZ pose_point;
          // pose_point.x = translation.x();
          // pose_point.y = translation.y();
          // pose_point.z = translation.z();
          // pose_tree.push_back(pose_point);   
          // tree.setInputCloud(pose_tree.makeShared());
          // // usleep(5e4);//5e4



          temp_cloud->clear();//! 清除submap
          // keyCloudInd++;//! 关键帧序号自增
          loop.sleep();
          std::cout << std::endl;
          }
        }
        // }
        // cloudInd++;
    // }
    // double mean_descriptor_time =
    //     std::accumulate(descriptor_time.begin(), descriptor_time.end(), 0) *
    //     1.0 / descriptor_time.size();
    // double mean_query_time =
    //     std::accumulate(querying_time.begin(), querying_time.end(), 0) * 1.0 /
    //     querying_time.size();
    // double mean_update_time =
    //     std::accumulate(update_time.begin(), update_time.end(), 0) * 1.0 /
    //     update_time.size();
    // std::cout << "Total key frame number:" << keyCloudInd
    //           << ", loop number:" << triggle_loop_num << std::endl;
    // std::cout << "Mean time for descriptor extraction: " << mean_descriptor_time
    //           << "ms, query: " << mean_query_time
    //           << "ms, update: " << mean_update_time << "ms, total: "
    //           << mean_descriptor_time + mean_query_time + mean_update_time
    //           << "ms" << std::endl;
    // LOG(INFO) << true_positive_recall << "/" << true_loop_frames << " = Recall rate: " << float(true_positive_recall) / true_loop_frames;
    // LOG(INFO) << true_positive_precision << "/" << true_in_reality << " = Precision rate: " << float(true_positive_precision) / true_in_reality;

    // LOG(INFO) << true_positive_recall_tmp << "/" << true_loop_frames_tmp << " = Recall rate: " << float(true_positive_recall_tmp) / true_loop_frames_tmp;
    // LOG(INFO) << true_positive_precision_tmp << "/" << true_in_reality_tmp << " = Precision rate: " << float(true_positive_precision_tmp) / true_in_reality_tmp;
    // break;
  // }

  return 0;
}