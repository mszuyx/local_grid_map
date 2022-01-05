#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "nav_msgs/OccupancyGrid.h"
#include <std_srvs/Empty.h> 
#include <tf/transform_listener.h>
#include <grid_map_core/BufferRegion.hpp>

using namespace grid_map;
//std::list<GridMap> mapQueue;

class localGridMap{
public:
  localGridMap();
private:
  // Declare sub & pub
  ros::NodeHandle node_handle_;
  ros::Subscriber map_sub_;
  ros::Publisher map_pub_;
  ros::Timer timer_;
  
  // Declare ROS params
  double map_width_;
  double map_length_;
  double delay_sec_;
  int queue_size_;
  std::string frame_id_;
  std::string base_frame_id_;
  
  tf::TransformListener tfListener;
    
  // Declare functions
  void local_map_callback_ (const nav_msgs::OccupancyGrid::ConstPtr& input_map);
  void timerCallback (const ros::TimerEvent&);
  void mapConcat (GridMap map1, GridMap map2);
};

localGridMap::localGridMap():node_handle_("~"){
  // Init ROS related
  ROS_INFO("Inititalizing Local Grid Map Node...");
 
  node_handle_.param("map_length_", map_length_, 12.0);
  ROS_INFO("map_length_: %f", map_length_);

  node_handle_.param("map_width_", map_width_, 12.0);
  ROS_INFO("map_width_: %f", map_width_);
  
  node_handle_.param("delay_sec_", delay_sec_, 0.0);
  ROS_INFO("delay_sec_: %f", delay_sec_);
  
  node_handle_.param("queue_size_", queue_size_, 10);
  ROS_INFO("queue_size_: %d", queue_size_);

  node_handle_.param<std::string>("frame_id_", frame_id_, "map"); 
  ROS_INFO("frame_id_: %s", frame_id_.c_str());
  
  node_handle_.param<std::string>("base_frame_id_", base_frame_id_, "base_link"); 
  ROS_INFO("base_frame_id_: %s", base_frame_id_.c_str());

  // Subscribe to occupancy grid
  map_sub_ = node_handle_.subscribe("/map_in", 5, &localGridMap::local_map_callback_, this);
  
  // Timer callback Init
  if (delay_sec_>0.0){
    timer_ = node_handle_.createTimer(ros::Duration(delay_sec_), &localGridMap::timerCallback, this);
  }
  // Publisher Init
  std::string grid_map;
  node_handle_.param<std::string>("local_grid_map_topic", grid_map, "/grid_map");
  ROS_INFO("local_grid_map_topic: %s", grid_map.c_str());
  map_pub_ = node_handle_.advertise<grid_map_msgs::GridMap>(grid_map, 1, true);
}

void localGridMap::timerCallback (const ros::TimerEvent&){
  std_srvs::Empty reset_Srv;
  ros::service::call("/octomap_server/reset", reset_Srv);
}

void localGridMap::local_map_callback_ (const nav_msgs::OccupancyGrid::ConstPtr& input_map){
  ROS_INFO("callback");
  
  //ros::WallTime startTime = ros::WallTime::now();
  
  // Get robot position related to map frame
  tf::StampedTransform RobotToOdomTf;
  try {
    tfListener.lookupTransform(frame_id_, base_frame_id_, input_map->header.stamp, RobotToOdomTf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }
  double base_x = RobotToOdomTf.getOrigin().getX();
  double base_y = RobotToOdomTf.getOrigin().getY();
  
  // Copy input occupancy grid info
  double res_map = double(input_map->info.resolution);
  int input_map_length = int(input_map->info.width);
  int input_map_width = int(input_map->info.height);
  //int sub_map_ori_x = int(std::round(((0.5*map_length_)/res_map) -(input_map_length+((input_map->info.origin.position.x)/res_map))));
  //int sub_map_ori_y = int(std::round((((0.5*map_width_)/res_map)-(input_map_width+((input_map->info.origin.position.y)/res_map)))));
  double raw_map_center_x = (input_map->info.origin.position.x)+(0.5*input_map_length*res_map);
  double raw_map_center_y = (input_map->info.origin.position.y)+(0.5*input_map_width*res_map);

  //std::cout << "resolution: " << input_map->info.resolution << " | map width: " << input_map->info.width << " | map height: " << input_map->info.height << " | sub_map_ori_x: " << sub_map_ori_x << " | Origin: " << input_map->info.origin << std::endl;

  // Reverse iteration is required because of different conventions between occupancy grid and grid map.
  Matrix data(input_map_length, input_map_width);
  for (std::vector<int8_t>::const_reverse_iterator iterator = input_map->data.rbegin();
      iterator != input_map->data.rend(); ++iterator) {
    Eigen::Index i = std::distance(input_map->data.rbegin(), iterator);  
    data(i) = *iterator != -1 ? *iterator : 50;
  }
  
  // Create grid map and set default value
  GridMap map({"traversability"});
  map.setFrameId(frame_id_);
  map.setGeometry(Length((input_map_length)*res_map, (input_map_width)*res_map), res_map);
  //map.setGeometry(Length(map_length_, map_width_), res_map);
  Position center(raw_map_center_x, raw_map_center_y);
  map.setPosition(center);
  map["traversability"].setConstant(50);
  
  /*
  int x_s;
  int y_s;
  if (input_map_length>std::round(map_length_/res_map)){
    x_s = std::round(0.5*(input_map_length-(map_length_/res_map)));
  }else{
    x_s = 0;
  }
  
  if (input_map_width>std::round(map_width_/res_map)){
    y_s = std::round(0.5*(input_map_width-(map_width_/res_map)));
  }else{
    y_s = 0;
  }
*/
  
  // Add data to grid map
  for (int x = 0; x < input_map_length; x++){
    for (int y = 0; y < input_map_width; y++){
      Index temp(x, y);
      map.at("traversability", temp) = data(x + input_map_length * y);
    }
  }

  // Move map based on robot position
  std::vector<BufferRegion> b;
  // b.push_back(BufferRegion(Index(0, 0), Size(50,50), BufferRegion::Quadrant::Undefined));
  Position robot_base(base_x, base_y);
  map.move(robot_base); //, std::vector<BufferRegion> newRegions);
  
  std::cout << base_x << base_y << std::endl;

  // Publish grid map.
  grid_map_msgs::GridMap map_msg;
  GridMapRosConverter::toMessage(map, map_msg);
  map_msg.info.header.stamp = input_map->header.stamp;
  map_pub_.publish(map_msg);
  //map.clearBasic();
  
  if (delay_sec_<=0.0){
    std_srvs::Empty reset_Srv;
    ros::service::call("/octomap_server/reset", reset_Srv);
  }
  //double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  //ROS_INFO("time used: %f sec)", total_elapsed);
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "local grid map");
    
  localGridMap node;
    
  ros::spin();
  return 0;
}


