#include "navigation_behaviours.h"

GoToPose::GoToPose(const std::string &name,
                  const BT::NodeConfig &config,
                  rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
  done_flag_=false;
}

BT::PortsList GoToPose::providedPorts()
{
  return {BT::InputPort<std::string>("loc")};
}

BT::NodeStatus GoToPose::onStart()
{
  // Get location key from port and read YAML file
  BT::Expected<std::string> loc = getInput<std::string>("loc");
  const std::string location_file = node_ptr_->get_parameter("location_file").as_string();

  //YAML::Node locations = YAML::LoadFile(location_file);
  YAML::Node locations = YAML::LoadFile("/home/arms/robocop_ws/src/RobocopPkg/tb_autonomy/include/locations.yaml");

  std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

  // setup action client
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);

  // make pose
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = pose[0];
  goal_msg.pose.pose.position.y = pose[1];

  tf2::Quaternion q;
  q.setRPY(0, 0, pose[2]);
  q.normalize(); // todo: why?
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  // send pose
  done_flag_ = false;
  action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Nav2\n");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning()
{
  if (done_flag_)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Goal reached\n", this->name().c_str());
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
    // If there is a result, we consider navigation completed.
  // bt_navigator only sends an empty message without status. Idk why though.

  if (result.result)
  {
    done_flag_ = true;
  }
}

/*BT::NodeStatus HumanDetection()
{
  bool resultReceived = 1;
  bool no_humans=1;

  if (!resultReceived)
  {
    RCLCPP_INFO(this->get_logger(), "Detection Module Failed");
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    if (no_humans)
    {
      RCLCPP_INFO(this->get_logger(), "Coast is clear sir");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Theres a human at [x,y,z]!!!!");
      
    }
    return BT::NodeStatus::SUCCESS;
  }


}*/
