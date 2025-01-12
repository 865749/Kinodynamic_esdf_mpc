#ifndef _SDF_MAP_HPP_
#define _SDF_MAP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <random>
#include <tuple>
#include <queue>

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.hpp>

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>

using namespace std;

struct Global_Map
{
  std::string path;
  Eigen::Vector2d map_origin_, map_size_;
  Eigen::Vector2d image_size;
  double resolution_, resolution_inv_;
  std::vector<char> occupancy_buffer_inflate_Global_Map;
  Eigen::Vector2i map_voxel_num_; // map range in index

};



struct MappingParameters
{
  Eigen::Vector2d map_origin_, map_size_;
  Eigen::Vector2d image_size;
  Eigen::Vector2d map_min_boundary_, map_max_boundary_; // map range in pos
  Eigen::Vector2d local_update_range_;
  int local_map_margin_;

  Eigen::Vector2i map_voxel_num_; // map range in index

  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  bool show_esdf_time_, show_occ_time_;

  string frame_id_;
};

struct MappingData
{
  Eigen::Vector2d laser_pos_, last_laser_pos_;
  Eigen::Quaterniond laser_q_, last_laser_q_;

  bool has_odom_, has_cloud_;
  std::vector<char> occupancy_buffer_inflate_;
  std::vector<Global_Map> Global_Maps;
  std::vector<char> occupancy_buffer_neg;
  std::vector<double> distance_buffer_neg_;
  std::vector<double> distance_buffer_all_;

  std::vector<double> distance_buffer_;

  std::vector<double> tmp_buffer1_;
  Eigen::Vector2i local_bound_min_, local_bound_max_;
  bool local_updated_, esdf_need_update_;
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int update_num_;
  bool use_global_map = false;
  int global_map_num = 0;
  int current_global_map = 0;
  inline int is_occupancy(int idx)
  {
    return occupancy_buffer_inflate_[idx] || (use_global_map ? Global_Maps[current_global_map].occupancy_buffer_inflate_Global_Map[idx] : 0);
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class SDFMap
{
public:
  SDFMap() {};
  ~SDFMap() {};

  enum
  {
    POSE_STAMPED = 1,
    ODOMETRY = 2,
    INVALID_IDX = -10000
  };
  typedef std::shared_ptr<SDFMap> Ptr;
  void getRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size);
  double getResolution();
  Global_Map load_map(std::string &path, const std::string &frame,int& map_buffer_size, MappingParameters &mp);
  void getSurroundPts(const Eigen::Vector2d &pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d &diff);
  void initMap(std::shared_ptr<rclcpp::Node> nh);

  inline double getDistance(const Eigen::Vector2d &pos);

  inline int getInflateOccupancy(Eigen::Vector2d pos);
  inline void posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id);
  inline void indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos);
  inline bool isInMap(const Eigen::Vector2d &pos);

  inline void boundIndex(Eigen::Vector2i &id);
  inline bool isInMap(const Eigen::Vector2i &idx);
  inline int toAddress(const Eigen::Vector2i &id);
  inline void setLocalMap(const int num)
  {
    if (num > md_.global_map_num || num < 0)
      return;
    md_.current_global_map = num;
  };
  inline int toAddress(int &x, int &y);

private:
  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf2_filter_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_sub_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  std::vector<std::shared_ptr<nav_msgs::msg::OccupancyGrid>> map_msgs;

  std::shared_ptr<rclcpp::Node> node_;
  MappingData md_;
  MappingParameters mp_;
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
  laser_geometry::LaserProjection projectoir_;
  // shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::TimerBase::SharedPtr esdf_timer_, vis_timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_, esdf_pub_;
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom);
  void laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser_msg);

  void publishMap();
  void publishESDF();

  void visCallback();

  void updateESDFCallback();
  void updateESDF2d();

  void resetBuffer(Eigen::Vector2d min_pos, Eigen::Vector2d max_pos);
  void publish_map()
  {
    // 发布消息
    if(!md_.use_global_map || md_.current_global_map > md_.global_map_num || md_.current_global_map <0)
      return;
    map_publisher_->publish(*(map_msgs[md_.current_global_map]));
    // RCLCPP_INFO(node_->get_logger(), "Map published.");
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
inline void SDFMap::posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id)
{
  for (int i = 0; i < 2; ++i)
    id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}
inline void SDFMap::indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos)
{
  for (int i = 0; i < 2; ++i)
    pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}
inline int SDFMap::toAddress(const Eigen::Vector2i &id)
{
  return id(0) * mp_.map_voxel_num_(1) + id(1);
}

inline int SDFMap::toAddress(int &x, int &y)
{
  return x * mp_.map_voxel_num_(1) + y;
}
inline void SDFMap::boundIndex(Eigen::Vector2i &id)
{
  Eigen::Vector2i id1;
  id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id = id1;
}
inline bool SDFMap::isInMap(const Eigen::Vector2i &idx)
{
  if (idx(0) < 0 || idx(1) < 0)
  {
    return false;
  }
  if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1)
  {
    return false;
  }
  return true;
}
inline double SDFMap::getDistance(const Eigen::Vector2d &pos)
{
  Eigen::Vector2i id;
  posToIndex(pos, id);
  boundIndex(id);

  return md_.distance_buffer_all_[toAddress(id)];
}
inline bool SDFMap::isInMap(const Eigen::Vector2d &pos)
{
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4)
  {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4)
  {
    return false;
  }
  return true;
}
inline int SDFMap::getInflateOccupancy(Eigen::Vector2d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector2i id;
  posToIndex(pos, id);

  return int(md_.is_occupancy(toAddress(id)));
}
#endif