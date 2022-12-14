#include <algorithm>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <sstream>
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>
#include <ur_kinematics/ur_kin.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
// Action Server headers
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// The Action Server "message type"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <unistd.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <osrf_gear/AGVControl.h>

//std_srvs::SetBool my_bool_var;
// my_bool_var.request.data = true;


#define MYSLEEP   10.0
#define SUBMITWAIT 20.0

std::vector<osrf_gear::Order> received_orders; // vector containing orders. Each order is a structure consisting of an order_id (string) and list of shipments (vector).
std::map<std::string, std::pair<std::string, int>> mat_bin;  // key = material_type, value = pair where .first is bin_name and .second is number of products requested.
std::map<std::string, osrf_gear::LogicalCameraImage> image_map; // key = bin_name, value = output of camera with poses of parts and pose of camera
sensor_msgs::JointState joint_states;
sensor_msgs::JointState previous_joint_state;
int count = 0;
bool first_time = true;
bool vacuum_state;
 
void start_competition(ros::NodeHandle & node){
 // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient begin_client = node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!begin_client.exists()) {
    ROS_WARN("Waiting for the competition to be ready...");
    begin_client.waitForExistence();
    ROS_WARN("Competition is now ready.");
  }
  ROS_WARN("Requesting competition start...");
  std_srvs::Trigger begin_comp;  // Combination of the "request" and the "response".
  bool service_call_succeeded;
  service_call_succeeded =  begin_client.call(begin_comp);  // Call the service to start the competition. Returns true if the client was able to initiate the service
  if (!service_call_succeeded) {
      ROS_ERROR("Competition service call failed! Goodness Gracious!!");
  } else if (!begin_comp.response.success) {  // Even if the call to the service succeeded, the service might not execute successfully. If execution failed, print why.
      ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
  } else { // if the service was called and executed successfully, inform user
      ROS_WARN("Competition service called succesfully! %s", begin_comp.response.message.c_str());
  }
}
 

void order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
  ROS_WARN_STREAM("Received order:\n" << *order_msg);  // Display incoming order messages, which contain a unit_ids (string) and lists of shipments (vector).  
  received_orders.push_back(*order_msg); // Add new orders to the end of the list of orders
}


// Adds the most recent camera image to the image_map where key is the correspoding camera source location.
void camera_callback(std::string bin_name, const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	osrf_gear::LogicalCameraImage new_image = *image_msg; 
	if(image_map.find(bin_name) != image_map.end()){
		image_map.insert(std::pair<std::string, osrf_gear::LogicalCameraImage>{bin_name, new_image});
	}else{
		image_map[bin_name] = new_image;
	}
}

// Define callbacks for each of the cameras
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

// Joint State Subscriber Callback, stores joint_states
void joint_state_callback(const sensor_msgs::JointState::ConstPtr & msg){
	joint_states = *msg;
	if(first_time){
		previous_joint_state = joint_states;
		first_time = false;
	}	
}

void vacuum_callback(const osrf_gear::VacuumGripperState::ConstPtr & msg){
	//vacuum_state = msg.attached;
}

// Method for identifying the name of the bin that a given product is in.
std::string find_bin(std::string material_type, ros::ServiceClient *client){
  osrf_gear::GetMaterialLocations location_msg; // location_msg can contain a material_type string and list of storage units where the material may be found
  location_msg.request.material_type = material_type;
  bool location_call_succeeded;
  location_call_succeeded = client->call(location_msg); // query the material location
  if (!location_call_succeeded){
    ROS_WARN("Material location returned failure.");
  } else{ // If the query succeeded, iterate through the list of storage units and return the name of the first storage unit containing the given material type that is not the belt.
      
    for (osrf_gear::StorageUnit unit : location_msg.response.storage_units){
      if(unit.unit_id != "belt"){
      	ROS_WARN("%s is found in storage unit %s", material_type.c_str(), unit.unit_id.c_str());
     	  	return unit.unit_id.c_str();
        }
        
    }
  }
  return "NONE"; // if no storage units contain the given material type, return "NONE"
}

int getTrajectory(trajectory_msgs::JointTrajectory *p_joint_trajectory, geometry_msgs::PoseStamped *goal_pose, double p_T_des[4][4], double p_q_des[8][6], double duration){

	// Desired pose of the end effector wrt the base_link.
	p_T_des[0][3] = goal_pose->pose.position.x;
	p_T_des[1][3] = goal_pose->pose.position.y;
	p_T_des[2][3] = goal_pose->pose.position.z+0.10;// above part
	p_T_des[3][3] = 1.0;
	// The orientation of the end effector so that the end effector is down.
	p_T_des[0][0] = 0.0; p_T_des[0][1] = -1.0; p_T_des[0][2] = 0.0;
	p_T_des[1][0] = 0.0; p_T_des[1][1] = 0.0; p_T_des[1][2]  = 1.0;
	p_T_des[2][0] = -1.0; p_T_des[2][1] = 0.0; p_T_des[2][2] = 0.0;
	p_T_des[3][0] = 0.0;  p_T_des[3][1] = 0.0; p_T_des[3][2] = 0.0;
	
	int num_sols = ur_kinematics::inverse((double *)&p_T_des[0][0], (double *)&p_q_des[0][0]);
	
	// Fill out the joint trajectory header.
	// Each joint trajectory should have an non-monotonically increasing sequence number.
	p_joint_trajectory->header.seq = count++;
	p_joint_trajectory->header.stamp = ros::Time::now(); // When was this message created.
	p_joint_trajectory->header.frame_id = "/world"; // Frame in which this is specified.
	
	

				
	// Set the names of the joints being used.  All must be present.
	p_joint_trajectory->joint_names.clear();
	p_joint_trajectory->joint_names.push_back("linear_arm_actuator_joint");
	p_joint_trajectory->joint_names.push_back("shoulder_pan_joint");
	p_joint_trajectory->joint_names.push_back("shoulder_lift_joint");
	p_joint_trajectory->joint_names.push_back("elbow_joint");
	p_joint_trajectory->joint_names.push_back("wrist_1_joint");
	p_joint_trajectory->joint_names.push_back("wrist_2_joint");
	p_joint_trajectory->joint_names.push_back("wrist_3_joint");

	// Set a start and end point.
	p_joint_trajectory->points.resize(3);
	// Set the start point to the current position of the joints from joint_states.
	p_joint_trajectory->points[0].positions.resize(p_joint_trajectory->joint_names.size());
	for (int indy = 0; indy < p_joint_trajectory->joint_names.size(); indy++) {
		for (int indz = 0; indz < joint_states.name.size(); indz++) {
			if (p_joint_trajectory->joint_names[indy] == joint_states.name[indz]) {
				p_joint_trajectory->points[0].positions[indy] = joint_states.position[indz];
				break;
			}
		}
	}
	
	
	// When to start (immediately upon receipt).
	p_joint_trajectory->points[0].time_from_start = ros::Duration(0.0);
	
	// Must select which of the num_sols solutions to use.  Just start with the first.
	int q_des_indx = 0;
	
	for(int i = 0; i < 7; i++){
		if(p_q_des[i][3] >= 3  && p_q_des[i][1] > 3 && p_q_des[i][2] < 3 && p_q_des[i][4] > 3){
			q_des_indx = i;
		}
	}
	
	
	ROS_WARN_STREAM(p_q_des[q_des_indx][0]);
	ROS_WARN_STREAM(p_q_des[q_des_indx][1]);
	ROS_WARN_STREAM(p_q_des[q_des_indx][2]);
	ROS_WARN_STREAM(p_q_des[q_des_indx][3]);
	ROS_WARN_STREAM(p_q_des[q_des_indx][4]);
	ROS_WARN_STREAM(p_q_des[q_des_indx][5]);
	
	// Set the end point for the movement
	p_joint_trajectory->points[1].positions.resize(p_joint_trajectory->joint_names.size());
	p_joint_trajectory->points[2].positions.resize(p_joint_trajectory->joint_names.size());
	// Set the linear_arm_actuator_joint from joint_states as it is not part of the inversen kinematics solution.
	
	
	
	p_joint_trajectory->points[1].positions[0] = joint_states.position[1];

	// The actuators are commanded in an odd order, enter the joint positions in the correct positions
	for (int indy = 0; indy < 6; indy++) {
		p_joint_trajectory->points[1].positions[indy + 1] = p_q_des[q_des_indx][indy]; 
	}
	
	p_T_des[2][3] += -0.08; //On Part
	
	num_sols = ur_kinematics::inverse((double *)&p_T_des[0][0], (double *)&p_q_des[0][0]);
	
	p_joint_trajectory->points[2].positions[0] = joint_states.position[1];
	// The actuators are commanded in an odd order, enter the joint positions in the correct positions
	for (int indy = 0; indy < 6; indy++) {
		p_joint_trajectory->points[2].positions[indy + 1] = p_q_des[q_des_indx][indy]; 
	}
	

	// How long to take for the movement.
	p_joint_trajectory->points[1].time_from_start = ros::Duration(duration);
	p_joint_trajectory->points[2].time_from_start = ros::Duration(duration + 1.0);
	return 0;
}

int move_actuator(trajectory_msgs::JointTrajectory *p_joint_trajectory, double linear_y, double duration){
	
	// Fill out the joint trajectory header.
	// Each joint trajectory should have an non-monotonically increasing sequence number.
	p_joint_trajectory->header.seq = count++;
	p_joint_trajectory->header.stamp = ros::Time::now(); // When was this message created.
	p_joint_trajectory->header.frame_id = "/world"; // Frame in which this is specified.
	
	

				
	// Set the names of the joints being used.  All must be present.
	p_joint_trajectory->joint_names.clear();
	p_joint_trajectory->joint_names.push_back("linear_arm_actuator_joint");
	p_joint_trajectory->joint_names.push_back("shoulder_pan_joint");
	p_joint_trajectory->joint_names.push_back("shoulder_lift_joint");
	p_joint_trajectory->joint_names.push_back("elbow_joint");
	p_joint_trajectory->joint_names.push_back("wrist_1_joint");
	p_joint_trajectory->joint_names.push_back("wrist_2_joint");
	p_joint_trajectory->joint_names.push_back("wrist_3_joint");

	// Set a start and end point.
	p_joint_trajectory->points.resize(2);
	// Set the start point to the current position of the joints from joint_states.
	p_joint_trajectory->points[0].positions.resize(p_joint_trajectory->joint_names.size());
	// Set the end point for the movement
	p_joint_trajectory->points[1].positions.resize(p_joint_trajectory->joint_names.size());
	for (int indy = 0; indy < p_joint_trajectory->joint_names.size(); indy++) {
		for (int indz = 0; indz < joint_states.name.size(); indz++) {
			if (p_joint_trajectory->joint_names[indy] == joint_states.name[indz]) {
				p_joint_trajectory->points[0].positions[indy] = joint_states.position[indz];
				p_joint_trajectory->points[1].positions[indy] = joint_states.position[indz];
				break;
			}
		}
	}
	// When to start (immediately upon receipt).
	p_joint_trajectory->points[0].time_from_start = ros::Duration(0.0);
	
	p_joint_trajectory->points[1].positions[0] = linear_y;
	// How long to take for the movement.
	p_joint_trajectory->points[1].time_from_start = ros::Duration(duration);
	return 0;
}

int getHome(trajectory_msgs::JointTrajectory *p_joint_trajectory,double p_q_des[6], std::string q_names[6]){
	// Fill out the joint trajectory header.
	// Each joint trajectory should have an non-monotonically increasing sequence number.
	p_joint_trajectory->header.seq = count++;
	p_joint_trajectory->header.stamp = ros::Time::now(); // When was this message created.
	p_joint_trajectory->header.frame_id = "/world"; // Frame in which this is specified.
	
	

				
	// Set the names of the joints being used.  All must be present.
	p_joint_trajectory->joint_names.clear();
	p_joint_trajectory->joint_names.push_back("linear_arm_actuator_joint");
	p_joint_trajectory->joint_names.push_back("shoulder_pan_joint");
	p_joint_trajectory->joint_names.push_back("shoulder_lift_joint");
	p_joint_trajectory->joint_names.push_back("elbow_joint");
	p_joint_trajectory->joint_names.push_back("wrist_1_joint");
	p_joint_trajectory->joint_names.push_back("wrist_2_joint");
	p_joint_trajectory->joint_names.push_back("wrist_3_joint");

	// Set a start and end point.
	p_joint_trajectory->points.resize(2);
	// Set the start point to the current position of the joints from joint_states.
	p_joint_trajectory->points[0].positions.resize(p_joint_trajectory->joint_names.size());
	for (int indy = 0; indy < p_joint_trajectory->joint_names.size(); indy++) {
		for (int indz = 0; indz < joint_states.name.size(); indz++) {
			if (p_joint_trajectory->joint_names[indy] == joint_states.name[indz]) {
				p_joint_trajectory->points[0].positions[indy] = joint_states.position[indz];
				break;
			}
		}
	}
	// When to start (immediately upon receipt).
	p_joint_trajectory->points[0].time_from_start = ros::Duration(0.0);
	
	// Set the end point for the movement
	p_joint_trajectory->points[1].positions.resize(p_joint_trajectory->joint_names.size());
	// Set the linear_arm_actuator_joint from joint_states as it is not part of the inversen kinematics solution.
	
	
	
	p_joint_trajectory->points[1].positions[0] = joint_states.position[1];

	for (int indy = 1; indy < p_joint_trajectory->joint_names.size(); indy++) {
		for (int indz = 0; indz < joint_states.name.size(); indz++) {
			if (p_joint_trajectory->joint_names[indy] == q_names[indz]) {
				p_joint_trajectory->points[1].positions[indy] = p_q_des[indz];
				break;
			}
		}
	}

	// How long to take for the movement.
	p_joint_trajectory->points[1].time_from_start = ros::Duration(5.0);
	return 0;
}

//Gets the linear actuatory position relating to the part distance from the 
double getLinear(geometry_msgs::PoseStamped *linear_pose, std::string bin){

	double linear = linear_pose->pose.position.y - sqrt(pow(1.2, 2) - pow(linear_pose->pose.position.x, 2));
	if(linear > 2.25){
		linear = 2.25;
	}
	if(linear < -2.1){
		linear = -2.1;
	}
	return linear;
}

int main(int argc, char **argv)
{
    ROS_ERROR("Waiting 10 seconds for ecse_373_ariac to complete");
    sleep(10);
    ROS_ERROR("Executing node.cpp");

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
   
   received_orders.clear();
   tf2_ros::Buffer tfBuffer;
   tf2_ros::TransformListener tfListener(tfBuffer);

	// Subscriber nodes for logical cameras.
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
	
	ros::ServiceClient submit_agv1_client = n.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
	ros::ServiceClient submit_agv2_client = n.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");
	
	//ros::ServiceClient submit_shipment_client = n.serviceClient<osrf_gear::SubmitShipment("/ariac/submit_shipment");
	
	//VACUUM SERVICE CLIENT
	ros::ServiceClient vacuum_client = n.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/arm1/gripper/control");
	ros::Subscriber vacuum_state_sub = n.subscribe("/ariac/arm1/gripper/state", 10, vacuum_callback);

	//JointState subscriber
	ros::Subscriber joint_state_sub = n.subscribe("/ariac/arm1/joint_states", 10, joint_state_callback);
	
	// Instantiate variables for use with the kinematic system.
	double T_pose[4][4];
	double q_pose[6];
	double T_des[4][4];
	double q_des[8][6];
	std::string q_pose_names[6];
	geometry_msgs::PoseStamped part_pose, goal_pose, bin_pose, first_pose, linear_pose;
	trajectory_msgs::JointTrajectory desired;
	trajectory_msgs::JointTrajectory trajectory;
	std::string camera_frame_name;
	double linear_y_pos = 0;
	std::string agv;
	
	control_msgs::FollowJointTrajectoryAction joint_trajectory_as;
	int status;
	geometry_msgs::TransformStamped tfStamped;
	
	
  //Subscribe to the '/ariac/orders' topic.
  ros::Subscriber orders_subscriber = n.subscribe("/ariac/orders", 10, order_callback);
  
  //Define client for GetMaterialLocations
  ros::ServiceClient location_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  location_client.waitForExistence();
  
  // Instantiate the Action Server client
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_as("ariac/arm1/arm/follow_joint_trajectory", true);
  
  ros::Rate loop_rate(10);
  start_competition(n);
  
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  int part_count = 0;
  
  while (ros::ok())
  {
		// if there are active orders, find where the parts are
		if(!received_orders.empty()){
		  // store the first order in received_orders as a separate variable, then remove that order from the list. 
			osrf_gear::Order order = received_orders.at(0);
      // Each order contains a set of shipments, which in turn have a list of products. For all shipments in the current order, identify the bin that each product is in. 
      // Print the bin number and coordinates of the selected product in the bin. Track how many of that product type have been ordered (for inventory purposes).
      
			for(osrf_gear::Shipment shipment: order.shipments){
				for(osrf_gear::Product product:shipment.products){
					std::string bin = find_bin(product.type.c_str(), &location_client);
						
					if(mat_bin.find(product.type.c_str()) != mat_bin.end()){
						mat_bin.at(product.type.c_str()).second += 1;
					}
					else{
						mat_bin.insert(std::pair<std::string, std::pair<std::string, int>>{product.type.c_str(), std::pair<std::string, int>{bin, 1}});
					}
					
					
					
				
					
					/*
					MOVE ARM BETWEEN BINS
					*/
					
					//MOVE LINEAR ACTUATOR
					bin_pose.pose.position.x = 0;
					bin_pose.pose.position.y = 0;
					bin_pose.pose.position.z = 0;
					
					//Set up linear actuator position
					try {
						tfStamped = tfBuffer.lookupTransform("arm1_linear_arm_actuator", "logical_camera_" + bin + "_frame",
						ros::Time(0.0), ros::Duration(1.0));
						ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
						tfStamped.child_frame_id.c_str());
					} catch (tf2::TransformException &ex) {
						ROS_ERROR("%s", ex.what());
					}
					tf2::doTransform(bin_pose, linear_pose, tfStamped);
					linear_y_pos = getLinear(&linear_pose, bin);
					
					//Move the linear actuator to where it's needed first
					status = move_actuator(&trajectory, linear_y_pos, 4.5);
					
					// Create the structure to populate for running the Action Server.
					// It is possible to reuse the JointTrajectory from above
					joint_trajectory_as.action_goal.goal.trajectory = trajectory;
					joint_trajectory_as.action_goal.header = trajectory.header;
					joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
					joint_trajectory_as.action_goal.goal_id.id = std::to_string(count);
						
					//ROS_WARN_STREAM("TRAJECTORY_AS: " << joint_trajectory_as);
						
					actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(10.0));
					ROS_WARN("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
					
					//MOVE ARM TO BETWEEN BIN POSITION
					// Retrieve the transformation
					
					try {
						tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_" + bin + "_frame",
						ros::Time(0.0), ros::Duration(1.0));
						ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
						tfStamped.child_frame_id.c_str());
					} catch (tf2::TransformException &ex) {
						ROS_ERROR("%s", ex.what());
					}
					
					tf2::doTransform(bin_pose, first_pose, tfStamped);
					if(bin == "bin1" || bin =="bin2" || bin == "bin3"){
						first_pose.pose.position.y += 0.40;
					}else{
						first_pose.pose.position.y += -0.40;
					}
					first_pose.pose.position.z += -0.30;
					first_pose.pose.orientation.w = 0.707;
					first_pose.pose.orientation.x = 0.0;
					first_pose.pose.orientation.y = 0.707;
					first_pose.pose.orientation.z = 0.0;
					//LINEAR ACTUATOR TRANSFORMATION
					
					
					
					status = getTrajectory(&trajectory, &first_pose, T_des, q_des, 5.0);
						
					//ROS_WARN_STREAM("TRAJECTORY: " << trajectory);
						
						
					// Create the structure to populate for running the Action Server.
					// It is possible to reuse the JointTrajectory from above
					joint_trajectory_as.action_goal.goal.trajectory = trajectory;
					joint_trajectory_as.action_goal.header = trajectory.header;
					joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
					joint_trajectory_as.action_goal.goal_id.id = std::to_string(count);
						
					//ROS_WARN_STREAM("TRAJECTORY_AS: " << joint_trajectory_as);
						
					state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(10.0));
					ROS_WARN("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
					
					
					/*
					MOVE ARM TO THE PART
					*/
					//Linear actuator first
						geometry_msgs::Pose pose = image_map[bin].models[part_count].pose;
						
						//START CONVERSION OF PART POSITION RELATIVE TO ACTUATOR
						
						bin_pose.pose.position.x = pose.position.x;
						bin_pose.pose.position.y = pose.position.y;
						bin_pose.pose.position.z = pose.position.z;
						
						//Set up linear actuator position
						try {
							tfStamped = tfBuffer.lookupTransform("arm1_linear_arm_actuator", "logical_camera_" + bin + "_frame",
							ros::Time(0.0), ros::Duration(1.0));
							ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
							tfStamped.child_frame_id.c_str());
						} catch (tf2::TransformException &ex) {
							ROS_ERROR("%s", ex.what());
						}
						tf2::doTransform(bin_pose, linear_pose, tfStamped);
						linear_y_pos = getLinear(&linear_pose, bin);
						
						//Move the linear actuator to where it's needed first
						status = move_actuator(&trajectory, linear_y_pos, 2.0);
						
						// Create the structure to populate for running the Action Server.
						// It is possible to reuse the JointTrajectory from above
						joint_trajectory_as.action_goal.goal.trajectory = trajectory;
						joint_trajectory_as.action_goal.header = trajectory.header;
						joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
						joint_trajectory_as.action_goal.goal_id.id = std::to_string(count);
							
						//ROS_WARN_STREAM("TRAJECTORY_AS: " << joint_trajectory_as);
							
						state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(10.0));
						ROS_WARN("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
							
					
						
						
						//Move down to the part.
						// find camera frame
						camera_frame_name = "logical_camera_" + bin + "_frame";
						
						// Retrieve the transformation
						try {
							tfStamped = tfBuffer.lookupTransform("arm1_base_link", camera_frame_name,
							ros::Time(0.0), ros::Duration(1.0));
							ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
							tfStamped.child_frame_id.c_str());
						} catch (tf2::TransformException &ex) {
							ROS_ERROR("%s", ex.what());
						}
						
						
						
						part_pose.pose = pose;
						
						tf2::doTransform(part_pose, goal_pose, tfStamped);
						
						// Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
						goal_pose.pose.orientation.w = 0.707;
						goal_pose.pose.orientation.x = 0.0;
						goal_pose.pose.orientation.y = 0.707;
						goal_pose.pose.orientation.z = 0.0;
						
						//ROS_WARN_STREAM("GOAL POSE: " << goal_pose);
						
						
						status = getTrajectory(&trajectory, &goal_pose, T_des, q_des, 1.0);
						
						//ROS_WARN_STREAM("TRAJECTORY: " << trajectory);
						
						
						// Create the structure to populate for running the Action Server.
						// It is possible to reuse the JointTrajectory from above
						joint_trajectory_as.action_goal.goal.trajectory = trajectory;
						joint_trajectory_as.action_goal.header = trajectory.header;
						joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
						joint_trajectory_as.action_goal.goal_id.id = std::to_string(count);
						
						//ROS_WARN_STREAM("TRAJECTORY_AS: " << joint_trajectory_as);
						
						state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(10.0));
						ROS_WARN("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
						
						
						
						//GRAB PART
						osrf_gear::VacuumGripperControl vacuum_control;  // Combination of the "request" and the "response".
						vacuum_control.request.enable = true;
  					vacuum_client.call(vacuum_control);  // Call service to start the vacuum.
						
						sleep(MYSLEEP);
						
						
					
					/*
						MOVE BETWEEN BINS BEFORE HOME
					*/
					
					
					//RAISE ARM BEFORE MOVING ACTUATOR
					goal_pose.pose.position.z += 0.30; //Raise above part
					
					status = getTrajectory(&trajectory, &goal_pose, T_des, q_des, 1.0);
						
					//ROS_WARN_STREAM("TRAJECTORY: " << trajectory);
						
						
					// Create the structure to populate for running the Action Server.
					// It is possible to reuse the JointTrajectory from above
					joint_trajectory_as.action_goal.goal.trajectory = trajectory;
					joint_trajectory_as.action_goal.header = trajectory.header;
					joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
					joint_trajectory_as.action_goal.goal_id.id = std::to_string(count);
						
					//ROS_WARN_STREAM("TRAJECTORY_AS: " << joint_trajectory_as);
						
					state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(10.0));
					ROS_WARN("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
					
					
					//START MOVING ARM TO BETWEEN THE BINS
					//actuator
					bin_pose.pose.position.x = 0;
					bin_pose.pose.position.y = 0;
					bin_pose.pose.position.z = 0;
					
					
					//Move arm to position
					// Retrieve the transformation
					
					try {
						tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_" + bin + "_frame",
						ros::Time(0.0), ros::Duration(1.0));
						ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
						tfStamped.child_frame_id.c_str());
					} catch (tf2::TransformException &ex) {
						ROS_ERROR("%s", ex.what());
					}
					
					tf2::doTransform(bin_pose, first_pose, tfStamped);
					if(bin == "bin1" || bin =="bin2" || bin == "bin3"){
						first_pose.pose.position.y += 0.40;
					}else{
						first_pose.pose.position.y += -0.40;
					}
					first_pose.pose.position.z += -0.30;
					first_pose.pose.orientation.w = 0.707;
					first_pose.pose.orientation.x = 0.0;
					first_pose.pose.orientation.y = 0.707;
					first_pose.pose.orientation.z = 0.0;
					
					
					
					status = getTrajectory(&trajectory, &first_pose, T_des, q_des, 5.0);
						
					//ROS_WARN_STREAM("TRAJECTORY: " << trajectory);
						
						
					// Create the structure to populate for running the Action Server.
					// It is possible to reuse the JointTrajectory from above
					joint_trajectory_as.action_goal.goal.trajectory = trajectory;
					joint_trajectory_as.action_goal.header = trajectory.header;
					joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
					joint_trajectory_as.action_goal.goal_id.id = std::to_string(count);
						
					//ROS_WARN_STREAM("TRAJECTORY_AS: " << joint_trajectory_as);
						
					state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(10.0));
					ROS_WARN("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
					
					
					/*
					HOME THE ROBOT ARM TO STARTING POSITION					
					*/
					
					q_pose[0] = previous_joint_state.position[1];
					q_pose[1] = previous_joint_state.position[2];
					q_pose[2] = previous_joint_state.position[3];
					q_pose[3] = previous_joint_state.position[4];
					q_pose[4] = previous_joint_state.position[5];
					q_pose[5] = previous_joint_state.position[6];
					
					q_pose_names[0] = previous_joint_state.name[1];
					q_pose_names[1] = previous_joint_state.name[2];
					q_pose_names[2] = previous_joint_state.name[3];
					q_pose_names[3] = previous_joint_state.name[4];
					q_pose_names[4] = previous_joint_state.name[5];
					q_pose_names[5] = previous_joint_state.name[6];
					
					status = getHome(&trajectory, q_pose, q_pose_names);
					
					// Create the structure to populate for running the Action Server.
					// It is possible to reuse the JointTrajectory from above
					joint_trajectory_as.action_goal.goal.trajectory = trajectory;
					joint_trajectory_as.action_goal.header = trajectory.header;
					joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
					joint_trajectory_as.action_goal.goal_id.id = std::to_string(count);
						
					//ROS_WARN_STREAM("TRAJECTORY_AS: " << joint_trajectory_as);
						
					state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(10.0));
					ROS_WARN("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
					
						
						/*
						MOVE TO THE AGV WITH PART
						*/
						// find camera frame
						agv = shipment.agv_id;
						if(shipment.agv_id == "any"){
							agv = "agv2";
						}
						
						camera_frame_name = "logical_camera_" + agv + "_frame";
						part_pose.pose = product.pose;
						
						//Set up linear actuator position
						try {
							tfStamped = tfBuffer.lookupTransform("arm1_linear_arm_actuator", camera_frame_name, ros::Time(0.0), ros::Duration(1.0));
							ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
							tfStamped.child_frame_id.c_str());
						} catch (tf2::TransformException &ex) {
							ROS_ERROR("%s", ex.what());
						}
						tf2::doTransform(part_pose, linear_pose, tfStamped);
						linear_y_pos = getLinear(&linear_pose, agv);
						
						//Move the linear actuator to where it's needed first
						status = move_actuator(&trajectory, linear_y_pos, 5.0);
						
						// Create the structure to populate for running the Action Server.
						// It is possible to reuse the JointTrajectory from above
						joint_trajectory_as.action_goal.goal.trajectory = trajectory;
						joint_trajectory_as.action_goal.header = trajectory.header;
						joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
						joint_trajectory_as.action_goal.goal_id.id = std::to_string(count);
							
						//ROS_WARN_STREAM("TRAJECTORY_AS: " << joint_trajectory_as);
							
						state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(10.0));
						ROS_WARN("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
					
					
						//Relative to Bin Position for AGV.
						bin_pose.pose.position.x = 0;
						bin_pose.pose.position.y = 0;
						bin_pose.pose.position.z = 0;
						
						
						//Move arm to position
						//Retrieve the transformation
						
						try {
							tfStamped = tfBuffer.lookupTransform("arm1_base_link", camera_frame_name,
							ros::Time(0.0), ros::Duration(1.0));
							ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
							tfStamped.child_frame_id.c_str());
						} catch (tf2::TransformException &ex) {
							ROS_ERROR("%s", ex.what());
						}
						
						tf2::doTransform(bin_pose, first_pose, tfStamped);
						first_pose.pose.position.x += -0.40;

						first_pose.pose.position.z += -0.40;
						first_pose.pose.orientation.w = 0.707;
						first_pose.pose.orientation.x = 0.0;
						first_pose.pose.orientation.y = 0.707;
						first_pose.pose.orientation.z = 0.0;
						//LINEAR ACTUATOR TRANSFORMATION
					
					
					
					status = getTrajectory(&trajectory, &first_pose, T_des, q_des, 5.0);
						
					//ROS_WARN_STREAM("TRAJECTORY: " << trajectory);
						
						
					// Create the structure to populate for running the Action Server.
					// It is possible to reuse the JointTrajectory from above
					joint_trajectory_as.action_goal.goal.trajectory = trajectory;
					joint_trajectory_as.action_goal.header = trajectory.header;
					joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
					joint_trajectory_as.action_goal.goal_id.id = std::to_string(count);
						
					//ROS_WARN_STREAM("TRAJECTORY_AS: " << joint_trajectory_as);
						
					state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(10.0));
					ROS_WARN("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
					
					
						
						//Move the arm to the AGV Part Pose position
						//START CONVERSION OF PART POSITION RELATIVE TO ARM
						std::string kit_frame = "";
						if(agv == "agv1"){
							kit_frame = "kit_tray_1";
						}else{
							kit_frame = "kit_tray_2";
						}
						// Retrieve the transformation
						try {
							tfStamped = tfBuffer.lookupTransform("arm1_base_link", kit_frame,
							ros::Time(0.0), ros::Duration(1.0));
							ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
							tfStamped.child_frame_id.c_str());
						} catch (tf2::TransformException &ex) {
							ROS_ERROR("%s", ex.what());
						}
						
						tf2::doTransform(part_pose, goal_pose, tfStamped);
						
						// Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
						goal_pose.pose.position.z += 0.02;
						goal_pose.pose.orientation.w = 0.707;
						goal_pose.pose.orientation.x = 0.0;
						goal_pose.pose.orientation.y = 0.707;
						goal_pose.pose.orientation.z = 0.0;
						
						//ROS_WARN_STREAM("GOAL POSE: " << goal_pose);
						
						
						status = getTrajectory(&trajectory, &goal_pose, T_des, q_des, 5.0);
						
						//ROS_WARN_STREAM("TRAJECTORY: " << trajectory);
						
						
						// Create the structure to populate for running the Action Server.
						// It is possible to reuse the JointTrajectory from above
						joint_trajectory_as.action_goal.goal.trajectory = trajectory;
						joint_trajectory_as.action_goal.header = trajectory.header;
						joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
						joint_trajectory_as.action_goal.goal_id.id = std::to_string(count);
						
						//ROS_WARN_STREAM("TRAJECTORY_AS: " << joint_trajectory_as);
						
						state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(10.0));
						ROS_WARN("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
						
						//Turn off the Vacuum Gripper
						/*
						DROP PART ON AGV IN ORIENTATION
						*/
						
						//DROP PART
						osrf_gear::VacuumGripperControl vacuum_control_off;  // Combination of the "request" and the "response".
						vacuum_control_off.request.enable = false;
  					vacuum_client.call(vacuum_control_off);  // Call service to start the vacuum.
						
						sleep(MYSLEEP);
						
						//Back to between AGV
						//Relative to Bin Position for AGV.
						bin_pose.pose.position.x = 0;
						bin_pose.pose.position.y = 0;
						bin_pose.pose.position.z = 0;
						
						
						//Move arm to position
						//Retrieve the transformation
						
						try {
							tfStamped = tfBuffer.lookupTransform("arm1_base_link", camera_frame_name,
							ros::Time(0.0), ros::Duration(1.0));
							ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
							tfStamped.child_frame_id.c_str());
						} catch (tf2::TransformException &ex) {
							ROS_ERROR("%s", ex.what());
						}
						
						tf2::doTransform(bin_pose, first_pose, tfStamped);
							first_pose.pose.position.x += -0.40;
					
						first_pose.pose.position.z += -0.40;
						first_pose.pose.orientation.w = 0.707;
						first_pose.pose.orientation.x = 0.0;
						first_pose.pose.orientation.y = 0.707;
						first_pose.pose.orientation.z = 0.0;
						//LINEAR ACTUATOR TRANSFORMATION
					
					
					
					status = getTrajectory(&trajectory, &first_pose, T_des, q_des, 5.0);
						
					//ROS_WARN_STREAM("TRAJECTORY: " << trajectory);
						
						
					// Create the structure to populate for running the Action Server.
					// It is possible to reuse the JointTrajectory from above
					joint_trajectory_as.action_goal.goal.trajectory = trajectory;
					joint_trajectory_as.action_goal.header = trajectory.header;
					joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
					joint_trajectory_as.action_goal.goal_id.id = std::to_string(count);
						
					//ROS_WARN_STREAM("TRAJECTORY_AS: " << joint_trajectory_as);
						
					state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(10.0));
					ROS_WARN("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
					
						
						
						/*
						HOME POSITION ARM
						*/
						q_pose[0] = previous_joint_state.position[1];
						q_pose[1] = previous_joint_state.position[2];
						q_pose[2] = previous_joint_state.position[3];
						q_pose[3] = previous_joint_state.position[4];
						q_pose[4] = previous_joint_state.position[5];
						q_pose[5] = previous_joint_state.position[6];
						
						q_pose_names[0] = previous_joint_state.name[1];
						q_pose_names[1] = previous_joint_state.name[2];
						q_pose_names[2] = previous_joint_state.name[3];
						q_pose_names[3] = previous_joint_state.name[4];
						q_pose_names[4] = previous_joint_state.name[5];
						q_pose_names[5] = previous_joint_state.name[6];
						
						status = getHome(&trajectory, q_pose, q_pose_names);
						
						// Create the structure to populate for running the Action Server.
						// It is possible to reuse the JointTrajectory from above
						joint_trajectory_as.action_goal.goal.trajectory = trajectory;
						joint_trajectory_as.action_goal.header = trajectory.header;
						joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
						joint_trajectory_as.action_goal.goal_id.id = std::to_string(count);
							
						//ROS_WARN_STREAM("TRAJECTORY_AS: " << joint_trajectory_as);
							
						state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(10.0));
						ROS_WARN("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
						
						
						part_count ++;
				}
				
				//SUBMIT SHIPMENT HERE
				
				osrf_gear::AGVControl submitsrv;
				submitsrv.request.shipment_type = shipment.shipment_type;
				if(agv == "agv1"){
					submit_agv1_client.call(submitsrv);
				}else{
					submit_agv2_client.call(submitsrv);
				}
				sleep(SUBMITWAIT);
				
			}
			received_orders.erase(received_orders.begin()); 
		}
		
		//ROS_WARN_STREAM_THROTTLE(10, joint_states);
		
	  loop_rate.sleep();
	  count++;

  }



  return 0;
}
