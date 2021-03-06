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
![STATES](https://github.com/Richz97/experimental_assignment1/blob/main/diagrams/STATES.png)
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

6. Go to catkin_ws/src/experimental_assignment1/launch and digit the command
```
roslaunch experimental_assignment1 simulation.launch
```
# Description of the execution
The execution of the project is redirected to the following video:

https://youtu.be/z_1iIqESyJQ

Refer to the description of the latter for more information on what is displayed on the screen.
# Working hypothesis and environment
As said at the beginning, the structure of this project is very simplified, an aspect that means that this structure can be improved and changed according to the required needs. All provided hints refer to a finite set of elements, which do not change over the course of execution. The winning hypothesis, fixed, can be modified at will, by modifying the appropriate line of code that refers to the winning identifier. Furthermore, the sleep functions present within the code have the objective of simulating both a computation and a movement within the environment by the robot.
## System's features
The whole system is able to manage hints, selected randomly, received randomly. In order to speed up the game, the system is able to recognize the hypotheses that have already been considered and rejected because they are not winning. This means that the robot does not have to go multiple times to the oracle, to verify a hint that refers to a hypothesis already considered. The randomity used in the system refers to a srand function and a function created specifically to generate random numbers. The srand function is just used to change the seed of this random function. This ensures that the random numbers generated are never the same, so you don't run into multiple unnecessary executions.
## System's limitations
The two limitations that are quite evident concern the simulation of the movement, which being a simulation does not refer to a graphical interface that allows in real time to see where the robot is actually located, and the fact that the possible hints and winning hypothesis are set from the start. A further limitation concerns the end of the game: in fact, when the game ends there is no direct start of a game session.
## System's technical improvements
In a subsequent implementation, referring to the aforementioned limitations, it could be possible to introduce a graphical interface that allows you to see, in real time, where the robot is actually located. Still with reference to the current limitations, a new game session could be started automatically at the end of the game without relaunching the project. Finally, the user could be allowed to modify the hints to be generated, and consequently also the winning hypothesis.
# Author and contacts
Author: Riccardo Zuppetti

Contacts: riccardo.zuppetti@icloud.com





