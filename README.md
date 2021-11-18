# Experimental Robotics Laboratory - Assignment 1
The purpose of this project is to obtain a simplified version of the Cluedo game. In particular, a robot will have to go around the 9 rooms that make up the map
(which in this case refers to the second edition of the game), in order to try to find as many hints as possible by discovering the details of the murder and then unmasking the culprit.
The verification of the various hypotheses, which correspond to the aggregation of several hints referring to the same identifier, is subjected to a so-called oracle.
## Compatibility
The project is developed using the provided docker container.
In the event that it is not possible to test the project on this image, it is also possible to use ROS Noetic on Ubuntu 20.04.2, after installing the ARMOR components on this operating system.
In any case, the project is compatible with ROS Noetic, and consequently may not work using a different ROS distribution.
## Expected Behaviour
Referring to the main purpose of the project, the robot should perform the following actions:
1. reach the various rooms that compose the entire environment
2. reached a room (randomly), the robot should find some hints, in order to constitute an hypothesis
3. formulated an hypothesis, the robot must be able to reach a designated position (the oracle), in order to verify the validity of this hypothesis
4. if the response given by the oracle is negative, the robot continues to move between the other rooms in order to constitute another hypothesis
## Project's features
The following proved necessary in the development of this project: the formalization of a behavioral architecture, so as to be able to impart to the robot different behaviors,
in relation to the action to be performed; the formalization of an environment in which the robot can move;
the use of the ontology provided in order to effectively manage the various hypotheses formulated.
# Description of the package
This package is composed by four nodes, two servers and one message.

The nodes are identifiable through the files inside the src and the scripts folder, and so we have:
* [robot_controller.cpp](https://github.com/Richz97/experimental_assignment1/blob/main/src/robot_controller.cpp)
* [motion_controller.cpp](https://github.com/Richz97/experimental_assignment1/blob/main/src/motion_controller.cpp)
* [oracle_controller.cpp](https://github.com/Richz97/experimental_assignment1/blob/main/src/oracle_controller.cpp)
* [hint_controller.py](https://github.com/Richz97/experimental_assignment1/blob/main/scripts/hint_controller.py)

Instead the servers are:
* [Motion.srv](https://github.com/Richz97/experimental_assignment1/blob/main/srv/Motion.srv)
* [Check.srv](https://github.com/Richz97/experimental_assignment1/blob/main/srv/Check.srv)

Finally, the message is:
* [Hypo.msg](https://github.com/Richz97/experimental_assignment1/blob/main/msg/Hypo.msg)

The node [robot_controller.cpp](https://github.com/Richz97/experimental_assignment1/blob/main/src/robot_controller.cpp) is the main node, the one that controls the robot inside the environment.
Here, there are the main function, that is encharged to establish in which state the robot is, and a functional callback, which is useful to retrieve the data that are published on the `/hypo`
topic. For this reason, we could say that this node subscribes to the `/hypo` topic, referring to [Hypo.msg](https://github.com/Richz97/experimental_assignment1/blob/main/msg/Hypo.msg).
The robot can basically be in two states, depending on whether it has to reach a random room or the oracle.
For this reason it was decided to give a Boolean value (0 or 1) to this state, in order to diversify the consequent behaviour of the robot. In the event that the robot has to reach a random room (state = 0),
a pseudo-random target is generated. After checking that the target does not correspond to the current position of the robot, we proceed with calling the `/change_room` server, which involves the performed movement. This aspect is also published on the `/reached` topic. From this it is easy to deduce that this node is able to publish data for the `/reached` topic and has a client for the `/change_room` server.
If, on the other hand, the robot has to reach the oracle (to verify a hypothesis) (state = 1), the target in this case is known and corresponds to the origin of the Cartesian axes, therefore (0, 0).
To refine the movement, the server `/change_room` is always called. Once the oracle has been reached, the `/check` server is called up, which verifies that the identifier of the hypothesis formulated is the same as that of the winning hypothesis. If successful, the game is over; in the event of a negative result, the robot is placed back in state 0. Therefore, this node also implements a client for the `/check` service.

The node [motion_controller.cpp](https://github.com/Richz97/experimental_assignment1/blob/main/src/motion_controller.cpp) implements a simple go_to_point function, which is simulated through a sleep function.
On top, this node also implements the `/change_room` service, used precisely to ensure that the robot can move in the environment.
This server, referring to [Motion.srv](https://github.com/Richz97/experimental_assignment1/blob/main/srv/Motion.srv), receives an empty request and reply with a Noolean variable set to True,
indicating that the movement has occurred.

The node [oracle_controller.cpp](https://github.com/Richz97/experimental_assignment1/blob/main/src/oracle_controller.cpp) includes, in addition to the main function, the send_hint function and the check_winner function. The send_hint function is a functional callback, and it's recalled when new data are published on the topic `/reached`,
topic on which the [robot_controller.cpp](https://github.com/Richz97/experimental_assignment1/blob/main/src/robot_controller.cpp) publishes. In this way, the send_hint is recalled when the robot has reached a
room. Reached the room, this function has the purpose of publishing the hints on the topic `/hint` (randomly), so as to be able to build an hypothesis. Therefore, this node is able to publish on the topic `/hint`.
The check_winner function is nothing more than the implementation of the `/check` server, whose purpose is to compare two identifiers and communicate whether they are the same or not.
In particular, referring to [Check.srv](https://github.com/Richz97/experimental_assignment1/blob/main/srv/Check.srv), this function, taking a string as a request, sends a boolean value as a response,
based on whether the two identifiers are equal or not. 

The node [hint_controller.py](https://github.com/Richz97/experimental_assignment1/blob/main/scripts/hint_controller.py) subscribes to the `/hint` topic, recalls the send_hint function and sequentially perform some checks on them in relation to the ontology. Initially the node checks if an hint is already present in the ontology and if not adds it. The same check is performed to see if the hints have already been saved in the ontology, and once this check is performed if the hint is new it checks if the hypothesis corresponding to that ID is complete and consistent. Finally it checks if the hypothesis has already been checked and sent. If it results as not yet sent, it publishes the hypothesis on the `/hypo` topic,
from which the [robot_controller.cpp](https://github.com/Richz97/experimental_assignment1/blob/main/src/robot_controller.cpp), which is the subscriber, can take it to the oracle.
On top, this node implements the client for the `/armor_interface_srv` server.

The server [Motion.srv](https://github.com/Richz97/experimental_assignment1/blob/main/srv/Motion.srv) takes as a request two floats that indicate the target that the robot must reach, and sends as a response a boolean value that indicates that the movement has occurred.

The server [Check.srv](https://github.com/Richz97/experimental_assignment1/blob/main/srv/Check.srv) takes as a request a string, that refers to an ID that must be checked by the oracle, and sends as a response a
boolean value that indicates if the compare had a positive or unsuccessful outcome.

The message [Hypo.msg](https://github.com/Richz97/experimental_assignment1/blob/main/msg/Hypo.msg) contain four strings that make up the hypothesis. Each hypothesis consists of the following four fields: ID, who, what and where. This message is used to store an hypothesis that needs to be tested.
## UML diagram
![UML](https://github.com/Richz97/experimental_assignment1/blob/main/diagrams/UML.png)
## Temporal diagram
![TEMPORAL](https://github.com/Richz97/experimental_assignment1/blob/main/diagrams/TEMPORAL.png)
## States diagram
![STATES](https://github.com/Richz97/experimental/blob/main/diagrams/STATES.png)
## Rqt-graph
Here we can see the rqt-graph of the entire system:

![Rqt-graph](https://github.com/Richz97/experimental_assignment1/blob/main/rqt_graph/rqt_graph.png)
# How to launch
1. Choice the catkin_ws in which there is the armor package

2. Move to cd catkin_ws/src/ros_multi_ontology_references/armor and digit `./gradlew deployApp` (only the first time)

3. Move to your catkin_ws/src folder and digit
```
git clone https://github.com/Richz97/experimental_assignment1.git
```
4. Move to catkin_ws/src/experimental_assignment1/scripts and digit
```
chmod +x hint_controller.py
```
5. Go to the root of the workspace, and from there launch `catkin_make`

6. Go to catkin_ws/src/experimental_assignment1 and digit the command
```
roslaunch experimental_assignment1 simulation.launch
```
# Description of the execution
// link_video

In this video it is possible to see what happens when we launch the simulation, which is launched by the above command.

Seeing the video, we could notice that, by default, the robot is in the state 0, i.e., the robot should reach a random target in the environment. As can be seen, the robot randomly chooses coordinates from the possible ones that correspond to possible rooms. Established the target, the robot goes to this room and once there finds a hint. Each time the hint is checked and handled by the hint node.

Instead, if the robot is the state 1 it has received a possible valid hypothesis, so it goes to the oracle room and proposes its hypothesis. Since the correct hypothesis is set as "ID3", obviously the oracle with the `/check` service will return a boolean False and the answer Wrong to the robot. Then the robot will resume searching fors hints and thus to the behaviour seen in the previous screenshot. In this screen you can also see that when a hypothesis has already been checked, the hint node will no longer handle the hint of that hypothesis avoiding checking one that has already been checked. This is communicated by the Mange Hypothesis: print which tells if: the hypothesis is complete or consistent, if it is not, or if it has already been checked. 

In this last screenshot we see the same situation as before but this time the hypothesis found is the winning one, so the oracle will return the boolean True and tell the robot Right. In this case, the script exits the loop and the programme ends.
# Working hypothesis and environment
This system is semplified in order to have a structure that can be changed easily and improved as needed. It is assumed that the hints are from a finite set of possibilities that does not change in time, the winning hypothesis is fixed and can be changed only manually by working on the code. The node that should implement the movement to a location is simply a wait of 0.5 seconds to simulate the motion without actually implementing it, we assume it takes some time to reach the room and that we don't have any obstacle.
## System's features
The system is really flexible and is able to handle random hints received at a random time.
In order to have a faster system it also saves the hypothesis that have already been made and it avoids repeating them, this prevents the robot from moving to home pointlessly every time it receives a hint from an hypothesis that is already been checked.
This system implements the randomness by using the rand function and the srand function. The srand is used to change the seed of the random function, it takes as input the time of the system and so the seed changes at every run of the code. By changing the seed we can ensure that the random number that are generated each time are different and so different situations can be tested by running the code multiple times.
## System's limitations
The biggest limitations are due to the lack of a real simulation of the robot and the environment. For the moment, in fact, we have a very abstract system where the behaviour of the robot and the environment are represented by several functions
## System's technical improvements
A future improvment will be for sure implementing the motion and provide the user with a graphical way to see where the robot is moving and what is the current state of the program in a more intuitive way.
Another improvment that can be made is the handling of the end of the robot process.
Also it is needed to add the possibility of changing the hints and the winning hypothesis from outside the code.
# Authors and contacts
Author: Riccardo Zuppetti
Contact: riccardo.zuppetti@icloud.com





