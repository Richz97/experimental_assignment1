/** 
*   @package experimental_assignment1
*
*   @file robot_controller.cpp
*   @brief Node that refers to the robot's behaviour
*
*   @author Riccardo Zuppetti
*   @version 1.0
*   @date 15/11/2021
*   @details
*  
*   Subscribes to: <BR>
*       /hypo
*
*   Publishes to: <BR>
*       /reached	    
*
*   Services: <BR>
*       None
*
*   Client Services: <BR>
*       /change_room
*       /check
*
*   Action Services: <BR>
*       None
*
*   Description: <BR>
*       The behaviour of the robot refers to the fact that this one could reach a random room (if it is not already inside)
*       or the oracle. It it receives an hypothesis, it goes to the oracle; if it not receives an hypothesis, it goes to 
*       a random room. For this reason, there is a subscriber to the /hypo topic. Each movement of the robot is accomplished 
*       by the /chage_room service. Reached a room, it publishes this information on the /reached topic. In case the robot
*       reachs the oracle, it recalls the /check service in order to verify if the hypothesis is correct or not. 
*/

#include "ros/ros.h"
#include "experimental_assignment1/Motion.h"
#include "std_msgs/Bool.h"
#include "experimental_assignment1/Check.h"
#include "experimental_assignment1/Hypo.h"
#include "std_msgs/String.h"
#include <string>
#include <ctime>

// global variables

int posx[9]={0,5,5,5,5,0,-5,-5,-5};
int posy[9]={5,5,2,-2,-5,-5,-5,0,5};
char *locations[9]={"Ballroom","Living room","Biliard room","Library","Study","Hall","Conservatory","Dining room","Kitchen"};

int current_state=0;

experimental_assignment1::Hypo temp_hypothesis;
experimental_assignment1::Motion temp_motion;
experimental_assignment1::Check temp_check;
std_msgs::Bool msg;


// function prototypes
double randMToN(double M, double N);
void clbk_hypo(const experimental_assignment1::Hypo x);

/** 
*   @brief main function
*   @param None
*   @return 0
*   The main function initializes the "robot_controller" node. It is useful
*   to initialize the various publisher(s), subscriber(s) and service(s). On top, there is also
*   the implementation of the behaviour of the robot (reach a random room/reach the oracle).
*/

int main(int argc, char **argv){
	
    ros::init(argc, argv, "robot_controller"); // initialization of the node
    
    ros::NodeHandle n;
    ros::NodeHandle n1;
    ros::NodeHandle n2;
    ros::NodeHandle n3;

    ros::ServiceClient cli1=n.serviceClient<experimental_assignment1::Motion>("/change_room"); // client for the /change_room service
    ros::Publisher pub1=n1.advertise<std_msgs::Bool>("/reached", 1000); // publisher for the /reached topic
    ros::ServiceClient cli2=n2.serviceClient<experimental_assignment1::Check>("/check"); // client for the /check service
    ros:: Subscriber sub1=n3.subscribe("/hypo", 1000, clbk_hypo); // subscriber to the /hypo topic
	
    int prec_location=0;
    int curr_location=0;
    srand(time(NULL)); // initialization of the seed (to generate random numbers)

    while(ros::ok()){
        ros::spinOnce();
        // the robot reach a random room (it is not already inside that room)
        if(current_state==0){
            prec_location=randMToN(0,8);
            // retrieve a position that is not equal to the current one
            while(prec_location==curr_location){
                prec_location=randMToN(0,8); 
            }
            curr_location=prec_location;
            // set the position (i.e. the room) that must be reached by the robot
            temp_motion.request.x=posx[curr_location];
            temp_motion.request.y=posy[curr_location];
            // print of the movement that the robot has to achieve
            std::cout << "\nGoing to the " << locations[curr_location] << " in x=" << temp_motion.request.x << " and y=" << temp_motion.request.y <<std::endl;
            cli1.call(temp_motion); // recall to the /change_room server
            msg.data=true; // movement performed
            pub1.publish(msg); // publish the message on the /reached topic
            sleep(1);
        }
        // the robot reach the oracle, in order to verify an hypothesis
        if(current_state==1){
            // set the position of the oracle (which corresponds to the origin of the cartesian axes)
            temp_motion.request.x = 0;
	        temp_motion.request.y = 0;
            std::cout << "\nGoing to the Oracle"<<std::endl;
            cli1.call(temp_motion); // recall to the /change_room server
            temp_check.request.id=temp_hypothesis.ID; // set the request for the /check server
            cli2.call(temp_check); // recall to the /check server
            // if the checked hypothesis is the correct one
            if (temp_check.response.ok==true){
                std::cout << "Correct hypothesis!" << std::endl;
                std::cout <<temp_hypothesis.who<< " with the "<<temp_hypothesis.what<<" in the "<<temp_hypothesis.where<< std::endl; // print of the correct hypothesis
                return 0;
            }
            // if the checked hypothesis is not the correct one
            else {
                std::cout << "Wrong hypothesis!" << std::endl;
                current_state=0; // update the behaviour of the robot (it must go to other rooms in order to formalize another hypothesis that then will be checked by the oracle)
            }
        }
    }
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
*   @brief function recalled when new data are available on the /hypo topic
*   @param const experimental_assignment1::Hypo x
*   @return None
*   When a new hypothesis is available, the robot change its behaviour (going to the oracle),
*   and it saves the hypothesis into a temp hypothesis, in order to have it verified by the oracle.
*/

void clbk_hypo(const experimental_assignment1::Hypo x){
    current_state=1; // change the behaviour of the robot
    temp_hypothesis=x;
}