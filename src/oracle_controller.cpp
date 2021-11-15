#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "experimental_assignment1/Check.h"
#include <string>
#include <ctime>
#include <cstdlib>

// global variables

ros::Publisher publisher;
ros::Subscriber subscriber;
std_msgs::String winner_id;
winner_id.data="ID3";

char *hints[15]={"ID1/where/Conservatory", "ID1/what/Dagger","ID1/where/Kitchen", 
	   "ID2/who/Plum", "ID2/what/Spanner","ID2/where/Hall",
	   "ID3/who/Miss.Scarlett", "ID3/what/Revolver","ID3/where/Lounge", 
	   "ID4/who/Green", "ID4/what/Candlestick","ID4/where/Library",
       "ID5/who/Col.Mustard", "ID5/what/Rope","ID5/who/Mrs.Peacock"};

void check_winner(experimental_assignment1::Check::Request &req, experimental_assignment1::Check::Response &res);
void send_hint(const std_msgs::Bool x);
double randMToN(double M, double N);

int main(int argc, char **argv){
    ros::init(argc, argv, "oracle_controller");
    
    ros::NodeHandle n;
    ros::NodeHandle n1;
    ros::NodeHandle n2;

    subscriber=n.subscribe("/reached", 1000, send_hint);
    publisher=n1.advertise<std_msgs::String>("/hint", 1000);
    ros::ServiceServer srv=n2.advertiseService("/check", check_winner);
    srand(time(NULL));
    ros::spin();
	return 0;
}


void send_hint(const std_msgs::Bool x){
    int index=randMToN(0,14);
    std_msgs::String msg;
    msg.data=hints[index];
    publisher.publish(msg);
}


double randMToN(double M, double N){     
	return M + (rand() / ( RAND_MAX / (N-M) ) ) ; 
}

void check_winner(experimental_assignment1::Check::Request &req, experimental_assignment1::Check::Response &res){
    if(req.id==winner_id.data){
        res.ok=true;
    }
    else{
        res.ok=false;
    }
}
