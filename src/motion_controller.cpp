#include "ros/ros.h"
#include "experimental_assignment1/Motion.h"
#include "std_msgs/Bool.h"

void simulate_motion(experimental_assignment1::Motion::Request &req, experimental_assignment1::Motion::Response &res);

int main(int argc, char **argv){
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle n;
    ros::ServiceServer srv_1=n.advertiseService("/change_room", simulate_motion);
    while(ros::ok()){
        ros::spinOnce();
    }
    return 0;
}

void simulate_motion(experimental_assignment1::Motion::Request &req, experimental_assignment1::Motion::Response &res){
    sleep(1);
    res.ok=true;
}
