#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
mavros_msgs::State current_state;
float z_current = 0.0;
uint8_t fix_type =0.0;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void alt_cb(const geometry_msgs::PoseStamped::ConstPtr& ksg){
    z_current= ksg->pose.position.z; 
}
void gps_fix_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    fix_type = msg->position_covariance_type;
}
int main(int argc, char **argv){

    ros::init(argc, argv, "takeoff_only_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber fix_sub = nh.subscribe<sensor_msgs::NavSatFix>
	    ("mavros/global_position/global",10,gps_fix_cb);
    ros::Subscriber Alt_sub = nh.subscribe<geometry_msgs::PoseStamped>
	    ("mavros/local_position/pose",1000,alt_cb);
    

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient cmd_client =nh.serviceClient<mavros_msgs::CommandLong>
	    ("mavros/cmd/command");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    ros::Rate fast_rate(1.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    mavros_msgs::CommandLong takeoff_srv;
    takeoff_srv.request.command = 22; //NAV_TAKEOFF
    takeoff_srv.request.param7 = 10;

    mavros_msgs::CommandLong land_srv;
    land_srv.request.command = 21; //NAV_LAND

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::SetMode RTL_set_mode;
    RTL_set_mode.request.custom_mode ="RTL";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
     while( ros::ok() && current_state.mode != "GUIDED"){
         if( set_mode_client.call(offb_set_mode)){

               ROS_INFO("GUIDED enabled ");
	       

            }else{
		ROS_INFO("WAITTING for GUIDED!");
		}
	ros::spinOnce();
        fast_rate.sleep();
	
       } 
    while(ros::ok() && (fix_type < 2)){
	ROS_INFO("GPS fix type %d", fix_type);
	ros::spinOnce();
	rate.sleep();
	}

    ROS_INFO("GPS fix type %d", fix_type);

    while(ros::ok() && !current_state.armed){
          if(arming_client.call(arm_cmd)){

               ROS_INFO("Vehicle armed");
	      

             }else{
		ROS_INFO("WAITING FOR ARMING");
		}
	 ros::spinOnce();
         fast_rate.sleep();
       }
    
  
   if(cmd_client.call(takeoff_srv)){
	ROS_INFO("Vehicle Takeoff"); 
     }else{
	ROS_INFO("Takeoff failed !!");
     }

   while(ros::ok() && z_current < 9.0){
		
	ROS_INFO("Current Altitude %f", z_current);
	ros::spinOnce();
        rate.sleep();
   	
    }
    ros::Duration(10.0).sleep();
    if(set_mode_client.call(RTL_set_mode)){
	ROS_INFO("Time to come back !");
     }else{
	ROS_INFO("LAND failed !!");
     }
    while(current_state.armed){
	ROS_INFO("I'm comming back");
	ros::spinOnce();
        fast_rate.sleep();
	}

    return 0;


}
