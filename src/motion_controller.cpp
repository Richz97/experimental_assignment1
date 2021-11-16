/** 
*   @package experimental_assignment1
*
*   @file motion_controller.cpp
*   @brief Node that simulates the motion of the robot among the various rooms 
*
*   @author Riccardo Zuppetti
*   @version 1.0
*   @date 15/11/2021
*   @details
*  
*   Subscribes to: <BR>
*       None
*
*   Publishes to: <BR>
*       None	    
*
*   Services: <BR>
*       /simulate_motion
*
*   Client Services: <BR>
*       None
*
*   Action Services: <BR>
*       None
*
*   Description: <BR>
*       The node implements a simulation of the movement of the robot. The whole movement is simulated
*       via a sleep function. Reached the specific room, the robot send a reply that refer to the fact
*       that the movement has been performed.
*/

#include "ros/ros.h"
#include "experimental_assignment1/Motion.h"
#include "std_msgs/Bool.h"

bool simulate_motion(experimental_assignment1::Motion::Request &req, experimental_assignment1::Motion::Response &res); // function prototype

/** 
*   @brief main function
*   @param None
*   @return 0
*   The main function initializes the "motion_controller" node
*/

int main(int argc, char **argv){
    ros::init(argc, argv, "motion_controller"); // definition of the node
    ros::NodeHandle n;
    ros::ServiceServer srv_1=n.advertiseService("/change_room", simulate_motion);
    while(ros::ok()){
        ros::spinOnce();
    }
    return 0;
}

/** 
*   @brief function that implements the simulation of the robot's movement
*   @param experimental_assignment1::Motion::Request &req
*   @return experimental_assignment1::Motion::Response &res
*/

bool simulate_motion(experimental_assignment1::Motion::Request &req, experimental_assignment1::Motion::Response &res){
    sleep(1); // sleep
    res.ok=true; // movement performed
    return 1;
}