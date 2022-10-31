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
 std::map<std::string, std::pair<std::string, int>> mat_bin;
 std::map<std::string, osrf_gear::LogicalCameraImage> image_map;
 
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

void camera_callback(std::string bin_name, const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	osrf_gear::LogicalCameraImage new_image = *image_msg;
	if(image_map.find(bin_name) != image_map.end()){
		image_map.insert(std::pair<std::string, osrf_gear::LogicalCameraImage>{bin_name, new_image});
	}else{
		image_map[bin_name] = new_image;
	}
}

void agv1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	camera_callback("agv1", image_msg);
}

void agv2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	camera_callback("agv2", image_msg);
}

void bin1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	camera_callback("bin1", image_msg);
}

void bin2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	camera_callback("bin2", image_msg);
}

void bin3_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	camera_callback("bin3", image_msg);
}

void bin4_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	camera_callback("bin4", image_msg);
}

void bin5_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	camera_callback("bin5", image_msg);
}

void bin6_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	camera_callback("bin6", image_msg);
}

void qc1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	camera_callback("qc1", image_msg);
}

void qc2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	camera_callback("qc2", image_msg);
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
        if(unit.unit_id.c_str() != "bin"){
        	ROS_INFO("%s is found in storage unit %s", material_type.c_str(), unit.unit_id.c_str());
        	return unit.unit_id.c_str();
        }
        
      }
  }
  return "NONE";
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


	// Subscribers for logical cameras.
	ros::Subscriber camera_agv1_sub = n.subscribe("/ariac/logical_camera_agv1", 10, agv1_callback);
	ros::Subscriber camera_agv2_sub = n.subscribe("/ariac/logical_camera_agv2", 10, agv2_callback);
	ros::Subscriber camera_bin1_sub = n.subscribe("/ariac/logical_camera_bin1", 10, bin1_callback);
	ros::Subscriber camera_bin2_sub = n.subscribe("/ariac/logical_camera_bin2", 10, bin2_callback);
	ros::Subscriber camera_bin3_sub = n.subscribe("/ariac/logical_camera_bin3", 10, bin3_callback);
	ros::Subscriber camera_bin4_sub = n.subscribe("/ariac/logical_camera_bin4", 10, bin4_callback);
	ros::Subscriber camera_bin5_sub = n.subscribe("/ariac/logical_camera_bin5", 10, bin5_callback);
	ros::Subscriber camera_bin6_sub = n.subscribe("/ariac/logical_camera_bin6", 10, bin6_callback);
	ros::Subscriber camera_qc1_sub = n.subscribe("/ariac/quality_control_sensor_1", 10, qc1_callback);
	ros::Subscriber camera_qc2_sub = n.subscribe("/ariac/quality_control_sensor_2", 10, qc2_callback);
	
  // Subscribe to the '/ariac/orders' topic.
  ros::Subscriber orders_subscriber = n.subscribe("/ariac/orders", 10, order_callback);
  
  // client for GetMaterialLocations
  ros::ServiceClient location_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  location_client.waitForExistence();
  
  ros::Rate loop_rate(10);
  start_competition(n);
  
  // find_bin("gear_part", location_client);
  
  while (ros::ok())
  {

		// if there are active orders, find where the parts are
		if(!received_orders.empty()){
				osrf_gear::Order order = received_orders.at(0);
				received_orders.erase(received_orders.begin());
				for(osrf_gear::Shipment shipment: order.shipments){
					for(osrf_gear::Product product:shipment.products){
						std::string bin = find_bin(product.type.c_str(), location_client);
						if(mat_bin.find(product.type.c_str()) != mat_bin.end()){
							mat_bin.at(product.type.c_str()).second += 1;
						}
						else{
							mat_bin.insert(std::pair<std::string, std::pair<std::string, int>>{product.type.c_str(), std::pair<std::string, int>{bin, 1}});
						}
						geometry_msgs::Point position = image_map[bin].models[0].pose.position;
						ROS_WARN_STREAM(product.type.c_str() << " is in bin: " << bin << " at position: x " << position.x << ", y " << position.y << ", z " << position.z);
					}
				}
		}
		
    ros::spinOnce();

    loop_rate.sleep();


  }



  return 0;
}
