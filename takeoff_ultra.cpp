#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <math.h>
#include <std_msgs/Bool.h>
#include "../../../devel/include/new_message/Ranges.h"

float z_current = 0.0;
uint8_t fix_type =0.0;
double Home_altitude =0.0;
mavros_msgs::State current_state;
new_message::Ranges range;
double lat_gps= 0.0;
double lon_gps= 0.0;
bool wet_sens = false;
bool safe_ok = false;
bool descend_cmd = false;
bool get_home = true;
mavros_msgs::GlobalPositionTarget home;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void range_cb(const new_message::Ranges::ConstPtr& msg){
    range = *msg;

}
void alt_cb(const geometry_msgs::PoseStamped::ConstPtr& ksg){
    z_current= ksg->pose.position.z; 
}
void gps_fix_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    fix_type = msg->position_covariance_type;
    Home_altitude = msg->altitude;
   if(get_home){
    	home.latitude = msg->latitude;
    	home.longitude = msg->longitude;
	get_home = false;
    }else{	
	lat_gps = msg->latitude;
    	lon_gps = msg->longitude;
	}
}
double Distance_calculator(double lat1, double  lon1, double  lat2,double lon2){
	double d_lat = lat1-lat2;
	double d_lon = lon1-lon2;
	const double R = 6371e3;
	double a = sin(d_lat/2)*sin(d_lat/2)+cos(lat1)*cos(lat2)*sin(d_lon/2)*sin(d_lon/2);
	double c =2*atan2(sqrt(a),sqrt(1-a));
	return 1.113195e5*sqrt((d_lat*d_lat) + (d_lon*d_lon));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "takeoff_ultra_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber range_sub = nh.subscribe<new_message::Ranges>
            ("/range_data",10,range_cb);
    ros::Publisher pumper_pub = nh.advertise<std_msgs::Bool>
	    ("/toggle_pump",10);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
	    ("mavros/setpoint_velocity/cmd_vel",10);
    ros::Publisher global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("mavros/setpoint_position/global", 10);
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
    takeoff_srv.request.param7 = 5; // Altitude Takeoff

    mavros_msgs::CommandLong land_srv;
    land_srv.request.command = 21; //NAV_LAND

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::SetMode loiter_set_mode;
    loiter_set_mode.request.custom_mode ="LAND";

    mavros_msgs::SetMode RTL_set_mode;
    RTL_set_mode.request.custom_mode ="RTL";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    std_msgs::Bool pump;
    pump.data =false;
    
    for(int i = 100; ros::ok() && i > 0; --i){
        ROS_INFO("GPS ALTITUDE, %f", Home_altitude);
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::TwistStamped z_vel;
    z_vel.twist.linear.z = -1;

//    mavros_msgs::GlobalPositionTarget home;
    mavros_msgs::GlobalPositionTarget target;
    double desired_altitude = 5;


    home.velocity.x = 0;
    home.velocity.y = 0;
    home.altitude = Home_altitude + desired_altitude;
    home.coordinate_frame=1;
    home.type_mask = 0b000011111111000;
    home.header.frame_id="base_link";
    
    target.latitude = 11.053180;
    target.longitude = 106.665682;
    target.velocity.x = 1;
    target.velocity.y = 1;
    target.altitude = Home_altitude + desired_altitude;
    target.coordinate_frame=5;
    target.type_mask = 0b000011111111000;
    target.header.frame_id="base_link";

    for(int i = 100; ros::ok() && i > 0; --i){
        
        global_pos_pub.publish(target);
        pumper_pub.publish(pump);
        ros::spinOnce();
        rate.sleep();
    }

   
    while( ros::ok() && current_state.mode != "GUIDED"){
         if( set_mode_client.call(offb_set_mode)){

               ROS_INFO("GUIDED enabled ");
	       

            }else{
		ROS_INFO("WAITTING for GUIDED!");
		}
	ros::spinOnce();
        fast_rate.sleep();
	
       } 
    while(ros::ok() && (fix_type < 1)){
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

   while(ros::ok() && z_current < 4.2){
		
	ROS_INFO("Current Altitude %f", z_current);
	ROS_INFO("Current Ultra Range %f", range.fusedRange);
	ros::spinOnce();
        rate.sleep();
      // Over Altitude Handle 
      // ***************************
      //****************************//
        if(z_current > 15.0){
         set_mode_client.call(loiter_set_mode);
         while(1){
          ROS_INFO("Dangerous Altitude !");
          ros::Duration(2.0).sleep();
         }
       }
      //***************************/
      //***************************/
    }
   int count = 0;
   float dist =100.0;
 
//******************************************
//******************************************
//***********Starting Sampling Task*********
//******************************************
//******************************************/
   ROS_INFO("Start Descending");
   ros::Duration(5.0).sleep();
   
   
   while(ros::ok()){

        if(range.Range1 > 200){
		wet_sens = true;
	}else wet_sens = false;

	dist = range.fusedRange;

	if(z_current > 2.0){
#ifdef _REAL_FLIGHT_
		if(dist < 60){
			ROS_INFO("Altitude Below Threshold");
			ROS_INFO("z Distance : %f", z_current);
			ROS_INFO("Range : %f", dist);
			safe_ok = false;
			descend_cmd = false;
		}else
#endif
			descend_cmd = true;
	}else{
	        
		if((wet_sens == false)){
		
			if(dist < 80){
				ROS_INFO("Altitude below Threshold!");
				ROS_INFO("z Distance : %f", z_current);
	                        ROS_INFO("Range : %f", dist);
				safe_ok = false;
				descend_cmd = false;
		
			}else{
			        ROS_INFO("Descending");
			        descend_cmd = true;
			}
		}else{
		         ROS_INFO("Water surface approached");
                         ros::Duration(10.0).sleep();
                         ROS_INFO("Start Sampling Task");
                         safe_ok = true;
                         descend_cmd = false;
                      
		}
	}
	
	if (descend_cmd){

	        z_vel.header.seq = count;
	       
	        z_vel.header.stamp = ros::Time::now();
                z_vel.twist.linear.z = -0.15;
	        z_vel.twist.linear.x = 0;
	        z_vel.twist.linear.y = 0;
	        cmd_vel_pub.publish(z_vel);
     //*************** 
     //***************/
                ROS_INFO("z Distance : %f", z_current);
	        ROS_INFO("Range : %f", dist);
      // Over Altitude Handle 
      // ***************************
      //****************************/
                if(z_current > 15.0){
                    set_mode_client.call(loiter_set_mode);
                while(1){
                        ROS_INFO("Dangerous Altitude !");
                        ros::Duration(2.0).sleep();
                }
               }
      //***************************
      //***************************
     //*************** 
     //***************/
	}else{
               
		z_vel.header.seq = count;
                z_vel.header.stamp = ros::Time::now();
                z_vel.twist.linear.z = 0.2;
                z_vel.twist.linear.x = 0;
                z_vel.twist.linear.y = 0;
                cmd_vel_pub.publish(z_vel);
		
		count++;

		ros::Duration(1.0).sleep();

	        z_vel.header.seq = count;
                z_vel.header.stamp = ros::Time::now();
                z_vel.twist.linear.z = 0;
                z_vel.twist.linear.x = 0;
                z_vel.twist.linear.y = 0;
                cmd_vel_pub.publish(z_vel);
     //*************** 
     //***************/
                ROS_INFO("z Distance : %f", z_current);
                ROS_INFO("Range : %f", dist);
     // Over Altitude Handle 
      // ***************************
      //****************************/
                if(z_current > 15.0){
                    set_mode_client.call(loiter_set_mode);
                while(1){
                        ROS_INFO("Dangerous Altitude !");
                        ros::Duration(2.0).sleep();
                }
               }
      //***************************
      //***************************
     //*************** 
     //***************/
		break;
	}
      
	count++;
   	ros::spinOnce();
        rate.sleep();
    }
ROS_INFO("Start to do something next !");
ros::Duration(2.0).sleep();
 if (safe_ok){ 
    pump.data=true;
    count = 0;
    wet_sens = false;
   while(ros::ok() && count < 20 ){
	if(range.Range1 > 550){
		wet_sens = true;
	}else wet_sens = false;
	if ((range.fusedRange < 60)&&(z_current > 10.0)){
		ROS_INFO("Dangerous case!");
         	break;
         }
   	 pumper_pub.publish(pump);
	 count++;
   	 ros::spinOnce();
         fast_rate.sleep();
	
    }

    ROS_INFO("Task Completed!");
    
    pump.data=false;
    count = 0;
   while(ros::ok() && count < 20){
      // Over Altitude Handle 
      // ***************************
      //****************************/
        if(z_current > 15.0){
         set_mode_client.call(loiter_set_mode);
         while(1){
          ROS_INFO("Dangerous Altitude !");
          ros::Duration(2.0).sleep();
         }
       }
      //***************************
      //***************************/
   	 pumper_pub.publish(pump);
	 count++;
   	 ros::spinOnce();
         rate.sleep();
    }
 
 }
   ROS_INFO("Start Climbing to 5 meter");
   ros::Duration(3.0).sleep();
//Climb to 5 meter//******
//************************
/************************/
	count = 0;
	while(ros::ok()){
		if(z_current < 4.2){
		
		z_vel.header.seq = count;
	       
	        z_vel.header.stamp = ros::Time::now();
                z_vel.twist.linear.z = 0.3;
	        z_vel.twist.linear.x = 0;
	        z_vel.twist.linear.y = 0;
	        cmd_vel_pub.publish(z_vel);
     //*************** 
     //***************
                ROS_INFO("z Distance : %f", z_current);
	        ROS_INFO("Range : %f", dist);
      // Over Altitude Handle 
      // ***************************
      //****************************
                if(z_current > 15.0){
                    set_mode_client.call(loiter_set_mode);
                while(1){
                        ROS_INFO("Dangerous Altitude !");
                        ros::Duration(2.0).sleep();
                }
               }
      //***************************
      //***************************
     //*************** 
     //***************

		}else{
			 z_vel.header.seq = count;
	       
	        	 z_vel.header.stamp = ros::Time::now();
               		 z_vel.twist.linear.z = 0;
	       		 z_vel.twist.linear.x = 0;
	       		 z_vel.twist.linear.y = 0;
	       		 cmd_vel_pub.publish(z_vel);
			 home.altitude = Home_altitude;
			 break;
		}
		
		count++;
   		ros::spinOnce();
   	        rate.sleep();

	}
	

ROS_INFO("Altitude Reached !!");
ros::Duration(3.0).sleep();

//******************************************
//******************************************
//*********Finishing Sampling Task**********
//******************************************
//******************************************
     dist = 5000;
     while (ros::ok() && dist > 20){
      // Over Altitude Handle 
      // ***************************
      //****************************
        if(z_current > 15.0){
         set_mode_client.call(loiter_set_mode);
         while(1){
          ROS_INFO("Dangerous Altitude !");
          ros::Duration(2.0).sleep();
         }
       }
      //***************************
      //***************************
	dist = Distance_calculator(lat_gps,lon_gps,home.latitude,home.longitude);
        home.header.seq = count;
	home.header.stamp = ros::Time::now();
	global_pos_pub.publish(home);
        ROS_INFO("Distance to home : %f", dist);
	count++;
   	ros::spinOnce();
        fast_rate.sleep();
     }

     ros::Duration(5.0).sleep();

     if(cmd_client.call(land_srv)){
	ROS_INFO("Time to come back !");
     }else{
	ROS_INFO("LAND failed !!");
     }
    while(current_state.armed){
      // Over Altitude Handle 
      // ***************************
      //****************************
        if(z_current > 15.0){
         set_mode_client.call(loiter_set_mode);
         while(1){
          ROS_INFO("Dangerous Altitude !");
          ros::Duration(2.0).sleep();
         }
       }
      //***************************
      //***************************
	ROS_INFO("I'm comming back");
	ROS_INFO("Lat: %f", home.latitude);
	ROS_INFO("Lon: %f", home.longitude);
	ros::spinOnce();
        fast_rate.sleep();
	}

    return 0;
}

