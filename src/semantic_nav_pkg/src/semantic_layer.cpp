#include "semantic_nav_pkg/semantic_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace semantic_nav_pkg
{

SemanticLayer::SemanticLayer()
{
}

// 1. Initialize: Start listening to the "Eyes"
void SemanticLayer::onInitialize()
{
  auto node = node_.lock();
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  // SUBSCRIBE to the topic your Python script publishes
  // "semantic_mask" must match what you write in Python later
  sub_ = node->create_subscription<sensor_msgs::msg::Image>(
    "/semantic_mask", 
    rclcpp::SensorDataQoS(), 
    std::bind(&SemanticLayer::semanticCallback, this, std::placeholders::_1));

  need_recalculation_ = false;
  current_ = true;
}

// 2. The Callback: When the AI sees something, save it.
void SemanticLayer::semanticCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // In a real system, you would project these pixels to the map frame (TF transform).
  // For this MVP, we assume the camera is looking down at the local map area.
  
  // Copy the raw pixel data (0-255 values) into our buffer
  semantic_cost_grid_ = msg->data; 
  need_recalculation_ = true;
}

// 3. Update Bounds: Tell Nav2 "I want to update the whole map"
void SemanticLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) return;

  // For this MVP, we say we update the entire local costmap
  // In a pro version, you'd calculate the exact field of view triangle
  auto * master = layered_costmap_->getCostmap();
  *min_x = master->getOriginX();
  *min_y = master->getOriginY();
  *max_x = master->getOriginX() + master->getSizeInMetersX();
  *max_y = master->getOriginY() + master->getSizeInMetersY();
}

// 4. Update Costs: The "Fusion" Logic (THE MONEY LINES)
void SemanticLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || semantic_cost_grid_.empty()) return;

  // We iterate over the map pixels
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();

  // Safety check to ensure mask matches map size (simplified for MVP)
  if (semantic_cost_grid_.size() != size_x * size_y) return;

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);
      
      // Get the value from the AI (The "Eyes")
      unsigned char semantic_value = semantic_cost_grid_[index];

      // --- YOUR CUSTOM LOGIC RULES ---
      
      // CASE A: PERSON detected (Pixel value 0 from YOLO)
      if (semantic_value == 0) { 
        master_array[index] = LETHAL_OBSTACLE; // Force Stop (Cost 254)
      }
      
      // CASE B: BUSH detected (Pixel value 1 from YOLO)
      // We set cost to 128 (High cost, but passable if necessary)
      else if (semantic_value == 1) {
         if (master_array[index] != LETHAL_OBSTACLE) {
            master_array[index] = 128; 
         }
      }
      
      // CASE C: ROAD/SAFE (Pixel value 2 from YOLO)
      // If the map thought it was unknown, we say "It's safe!"
      else if (semantic_value == 2) {
         if (master_array[index] == NO_INFORMATION) {
            master_array[index] = FREE_SPACE;
         }
      }
    }
  }
}

void SemanticLayer::onFootprintChanged()
{
  need_recalculation_ = true;
}

}  // namespace semantic_nav_pkg