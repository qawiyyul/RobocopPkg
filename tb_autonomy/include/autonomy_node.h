#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "navigation_behaviours.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <tf2/LinearMath/Quaternion.h>

class AutonomyNode : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree_;

public:
    explicit AutonomyNode(const std::string &node_name);
    void setup();
    void create_behaviour_tree();
    void update_behaviour_tree();
};

