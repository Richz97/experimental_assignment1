/** 
*   @package experimental_assignment1
*
*   @file oracle_controller.cpp
*   @brief Node that sends random hints and checks if the received hypothesis is the correct one
*
*   @author Riccardo Zuppetti
*   @version 1.0
*   @date 15/11/2021
*   @details
*  
*   Subscribes to: <BR>
*       /reached
*
*   Publishes to: <BR>
*       /hint	    
*
*   Services: <BR>
*       /check
*
*   Client Services: <BR>
*       None
*
*   Action Services: <BR>
*       None
*
*   Description: <BR>
*       The node allow to publish on the /hint topic, randombly, an hint that belongs to a list
*       of all the possible hints, in which there is one that is correct. It subscribes to the /reached topic,
*       and, on top, it implements a server that checks if an hypothesis is the correct one.
*/


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

// possible hints
char *hints[15]={"ID1/where/Conservatory", "ID1/what/Dagger","ID1/where/Kitchen", 
	   "ID2/who/Plum", "ID2/what/Spanner","ID2/where/Hall",
	   "ID3/who/Miss.Scarlett", "ID3/what/Revolver","ID3/where/Lounge", 
	   "ID4/who/Green", "ID4/what/Candlestick","ID4/where/Library",
       "ID5/who/Col.Mustard", "ID5/what/Rope","ID5/who/Mrs.Peacock"};

// function prototypes
bool check_winner(experimental_assignment1::Check::Request &req, experimental_assignment1::Check::Response &res);
void send_hint(const std_msgs::Bool x);
double randMToN(double M, double N);

/** 
*   @brief main function
*   @param None
*   @return 0
*   The main function initializes the "oracle_controller" node. On top, it is useful
*   to initialize the various publisher(s), subscriber(s) and service(s). 
*/

int main(int argc, char **argv){
    ros::init(argc, argv, "oracle_controller"); // definition of the node
    
    ros::NodeHandle n;
    ros::NodeHandle n1;
    ros::NodeHandle n2;

    subscriber=n.subscribe("/reached", 1000, send_hint); // subscriber to the topic /reached
    publisher=n1.advertise<std_msgs::String>("/hint", 1000); // publisher for the topic /hint
    ros::ServiceServer srv=n2.advertiseService("/check", check_winner); // service for the topic /check
    srand(time(NULL)); // seed initialization (useful for generating random numbers)
    ros::spin();
    return 0;
}

/** 
*   @brief function that allows to publish hints on the /hint topic
*   @param const std_msgs::Bool x
*   @return None
*   The function generates a random number in order to establish which element
*   that belongs to the array of possible hints should be published on the /hint topic.
*/

void send_hint(const std_msgs::Bool x){
    int index=randMToN(0,14); // generates random number
    std_msgs::String msg; // initialization of a message that will be published
    msg.data=hints[index];
    publisher.publish(msg);
}

/** 
*   @brief function that generates a random number that belongs to the interval [M, N]
*   @param double M (minimum number)
*   @param double N (maximum number)
*   @return double random
*/

double randMToN(double M, double N){     
	return M + (rand() / ( RAND_MAX / (N-M) ) ) ; 
}

/** 
*   @brief function that checks if an hypothesis is the correct one or not
*   @param experimental_assignment1::Check::Request &req
*   @param experimental_assignment1::Check::Response &res
*   @return None
*   In particular, the check is carried out on the ID field of the hypothesis.
*/

bool check_winner(experimental_assignment1::Check::Request &req, experimental_assignment1::Check::Response &res){
    std_msgs::String winner_id;
    winner_id.data="ID3"; // ID of the correct hypothesis
    if(req.id==winner_id.data){ // compare of the two IDs
        res.ok=true;
    }
    else{
        res.ok=false;
    }
}
