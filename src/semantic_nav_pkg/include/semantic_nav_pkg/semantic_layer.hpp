#ifndef SEMANTIC_LAYER_HPP_
#define SEMANTIC_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace semantic_nav_pkg
{

// This class inherits from the standard Nav2 Layer
class SemanticLayer : public nav2_costmap_2d::Layer
{
public:
  SemanticLayer();

  // 1. Lifecycle Methods (Standard ROS 2 State Machine)
  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);
  virtual void reset() { return; }
  virtual void onFootprintChanged();

private:
  // 2. The Subscriber (Listening to your Python Node)
  void semanticCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  // 3. Variables to store the semantic data
  double last_robot_x_, last_robot_y_;
  bool need_recalculation_;
  
  // Storage for the semantic mask (from Python)
  // We use 0-255 values where 255 = Lethal
  std::vector<unsigned char> semantic_cost_grid_; 
};

}  // namespace semantic_nav_pkg

#endif  // SEMANTIC_LAYER_HPP_