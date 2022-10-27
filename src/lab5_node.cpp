#include <algorithm>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include "std_msgs/String.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include <sstream>
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>

//std_srvs::SetBool my_bool_var;
// my_bool_var.request.data = true;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
 std::vector<osrf_gear::Order> received_orders;
 
 void start_competition(ros::NodeHandle & node){
 // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient begin_client =
    node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!begin_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    begin_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger begin_comp;  // Combination of the "request" and the "response".
  bool service_call_succeeded;
  service_call_succeeded =  begin_client.call(begin_comp);  // Call the start Service.
  if (!service_call_succeeded) {
      ROS_ERROR("Competition service call failed! Goodness Gracious!!");
  } else if (!begin_comp.response.success) {  // If not successful, print out why.
      ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
  } else {
      ROS_INFO("Competition service called succesfully! %s", begin_comp.response.message.c_str());
  }
 }
 
 void order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
  ROS_INFO_STREAM("Received order:\n" << *order_msg);
  received_orders.push_back(*order_msg);
}

//MAKE THIS A std::string
std::string find_bin(std::string material_type, ros::ServiceClient client){
  osrf_gear::GetMaterialLocations location_msg;
  location_msg.request.material_type = material_type;
  bool location_call_succeeded;
  location_call_succeeded = client.call(location_msg);
  if (!location_call_succeeded){
    ROS_WARN("Material location returned failure.");
  } else{
      for (osrf_gear::StorageUnit unit : location_msg.response.storage_units){
        ROS_INFO("%s is found in storage unit %s", material_type.c_str(), unit.unit_id.c_str());
      }
  }
  return "Done";
 }
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the .
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // Subscribe to the '/ariac/orders' topic.
  ros::Subscriber orders_subscriber = n.subscribe("/ariac/orders", 10, order_callback);
  
  // 
  ros::ServiceClient location_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  location_client.waitForExistence();
  
  ros::Rate loop_rate(10);
  start_competition(n);
  
  find_bin("gear_part", location_client);
  
  while (ros::ok())
  {


    ros::spinOnce();

    loop_rate.sleep();


  }



  return 0;
}
