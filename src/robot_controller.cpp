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

double randMToN(double M, double N);
void clbk_hypo(const experimental_assignment1::Hypo x);

int main(int argc, char **argv){
    int prec_location=0;
    int curr_location=0;
    ros::init(argc, argv, "robot_controller");
    
    ros::NodeHandle n;
	ros::NodeHandle n1;
    ros::NodeHandle n2;
    ros::NodeHandle n3;

    ros::ServiceClient cli1=n.serviceClient<experimental_assignment1::Motion>("/change_room");
    ros::Publisher pub1=n1.advertise<std_msgs::Bool>("/reached", 1000);
    ros::ServiceClient cli2=n2.serviceClient<experimental_assignment1::Check>("/check");
    ros:: Subscriber sub1=n3.subscribe("/hypo", 1000, clbk_hypo);

    srand(time(NULL));

    while(ros::ok()){
        ros::spinOnce();
        if(current_state==0){
            prec_location=randMToN(0,8);
            while(prec_location==curr_location){
                prec_location=randMToN(0,8);
            }
            curr_location=prec_location;
            temp_motion.request.x=posx[curr_location];
            temp_motion.request.y=posy[curr_location];
            std::cout << "\nGoing to the position: x= " << temp_motion.request.x << " y= " <<temp_motion.request.y <<"  " << locations[index] <<std::endl;
            cli1.call(temp_motion);
            msg.data=true;
            pub1.publish(msg);
            sleep(1);
        }
        if(current_state==1){
            temp_motion.request.x = 0;
	        temp_motion.request.y = 0;
            std::cout << "\nGoing to the Oracle"<<std::endl;
            cli1.call(temp_motion);
            temp_check.request.id=temp_hypothesis.ID;
            cli2.call(temp_check);
            if (temp_check.response.ok==true){
                std::cout << "Correct hypothesis!" << std::endl;
                std::cout <<temp_hypothesis.who<< " with the "<<temp_hypothesis.what<<" in the "<<temp_hypothesis.where<< std::endl;
                return 0;
            }
            else {
                std::cout << "Wrong hypothesis!" << std::endl;
                current_state=0;
            }
        }
    }
}


double randMToN(double M, double N){     
	return M + (rand() / ( RAND_MAX / (N-M) ) ) ; 
}


void clbk_hypo(const experimental_assignment1::Hypo x){
    current_state=1;
    temp_hypothesis=x;
}
