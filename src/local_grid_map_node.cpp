#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "nav_msgs/OccupancyGrid.h"
#include <std_srvs/Empty.h> 
//#include "std_msgs/Header.h"

using namespace grid_map;
GridMap map({"traversability"});

class localGridMap{
public:
  localGridMap();
private:
  // Declare sub & pub
  ros::NodeHandle node_handle_;
  ros::Subscriber map_sub_;
  ros::Publisher map_pub_;
  
  // Declare ROS params
  double map_width_;
  double map_length_;
  std::string frame_id_;
    
  // Declare functions
  void local_map_callback_ (const nav_msgs::OccupancyGrid::ConstPtr& input_map);
};

localGridMap::localGridMap():node_handle_("~"){
  // Init ROS related
  ROS_INFO("Inititalizing Local Grid Map Node...");
 
  node_handle_.param("map_length_", map_length_, 10.0);
  ROS_INFO("map_length_: %f", map_length_);

  node_handle_.param("map_width_", map_width_, 10.0);
  ROS_INFO("map_width_: %f", map_width_);

  node_handle_.param<std::string>("frame_id_", frame_id_, "map"); 
  ROS_INFO("frame_id_: %s", frame_id_.c_str());

  // Subscribe to occupancy grid
  map_sub_ = node_handle_.subscribe("/map_in", 5, &localGridMap::local_map_callback_, this);
    
  // Publisher Init
  std::string grid_map;
  node_handle_.param<std::string>("local_grid_map_topic", grid_map, "/grid_map");
  ROS_INFO("local_grid_map_topic: %s", grid_map.c_str());
  map_pub_ = node_handle_.advertise<grid_map_msgs::GridMap>(grid_map, 1, true);

  map.setFrameId(frame_id_);
}

void localGridMap::local_map_callback_ (const nav_msgs::OccupancyGrid::ConstPtr& input_map){
  //std::cout << "resolution: " << input_map->info.resolution << " | map width: " << input_map->info.width << " | map height: " << input_map->info.height << " | Origin: " << input_map->info.origin << std::endl;
  //ros::WallTime startTime = ros::WallTime::now();
  
  // Copy input occupancy grid info
  double res_map = double(input_map->info.resolution);
  int input_map_length = int(input_map->info.width);
  int input_map_width = int(input_map->info.height);
  //int sub_map_ori_x = int(std::round((((map_length_)/res_map)-input_map_length))); // half map
  //int sub_map_ori_y = int(std::round((((0.5*map_width_)/res_map)-(input_map_width+((input_map->info.origin.position.y)/res_map)))));
  int sub_map_ori_x = int(std::round(((0.5*map_length_)/res_map) -(input_map_length+((input_map->info.origin.position.x)/res_map))));
  int sub_map_ori_y = int(std::round((((0.5*map_width_)/res_map)-(input_map_width+((input_map->info.origin.position.y)/res_map)))));
  // Reverse iteration is required because of different conventions between occupancy grid and grid map.
  Matrix data(input_map_length, input_map_width);
  for (std::vector<int8_t>::const_reverse_iterator iterator = input_map->data.rbegin();
      iterator != input_map->data.rend(); ++iterator) {
    Eigen::Index i = std::distance(input_map->data.rbegin(), iterator);
    data(i) = *iterator != -1 ? *iterator : 50;
  }

  // Create grid map and set default value
  map.setGeometry(Length(map_length_, map_width_), res_map);
  //Position center((map_length_*0.5)+input_map->info.origin.position.x, 0.0); // half map
  Position center(0.0, 0.0);
  map.setPosition(center);
  map["traversability"].setConstant(50);
  
  /*
  if (sub_map_ori_x+input_map_length > std::round((map_length_)/res_map)){
  std::cout << "Bad x offset value: " << sub_map_ori_x+input_map_length << std::endl;
  }
  if (sub_map_ori_y+input_map_width > std::round((map_width_)/res_map)){
  std::cout << "Bad y offset value: " << sub_map_ori_y+input_map_width << std::endl;
  }
  */
  
  // Add data to grid map
  for (int x = 0; x < input_map_length; x++){
    for (int y = 0; y < input_map_width; y++){
      Index temp(sub_map_ori_x+x, sub_map_ori_y+y);
      map.at("traversability", temp) = data(x + input_map_length * y);
    }
  }

  /* 
    Index submapStartIndex(sub_map_ori_x, sub_map_ori_y);
    Index submapBufferSize(input_map_length, input_map_width);
    for (SubmapIterator iterator(map, submapStartIndex, submapBufferSize); !iterator.isPastEnd(); ++iterator) {
    size_t i = std::distance(submapStartIndex, iterator);
    map.at("traversability", *iterator) = data(i);//mapDataConvert(input_map->data[*iterator]);
    grid_count += 1;
    //std::cout << "iterator: " << *iterator(0) <<std::endl;
  }
  */

  /*
  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
    Position position;
    map.getPosition(*it, position);
    map.at("traversability", *it) = float(-0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x());
  }
  */

  // Publish grid map.
  grid_map_msgs::GridMap map_msg;
  GridMapRosConverter::toMessage(map, map_msg);
  map_msg.info.header.stamp = input_map->header.stamp;
  map_pub_.publish(map_msg);
  //map.clearBasic();
  
  std_srvs::Empty reset_Srv;
  ros::service::call("/octomap_server/reset", reset_Srv);
  
  //double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  //ROS_INFO("time used: %f sec)", total_elapsed);
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "local grid map");
    
  localGridMap node;
    
  ros::spin();
  return 0;
}


