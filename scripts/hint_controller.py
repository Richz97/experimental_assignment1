#! usr/bin/env python2

##  @package experimental_assignment1
#
#   @file hint_controller.py
#   @brief This program elaborates all the received hints
#
#   @author Riccardo Zuppetti
#   @version 1.0
#   @date 15/11/2021
#   @details
#  
#   Subscribes to: <BR>
#       /hint
#
#   Publishes to: <BR>
#       /hypo	    
#
#   Services: <BR>
#       None
#
#   Client Services: <BR>
#       armor_interface_srv
#
#   Action Services: <BR>
#       None
#
#   Description: <BR>
#       The node subscribes to the /hint topic, in order to operate on them. If the hint is not already present
#       in the ontology, it adds to this one. If it is the first time the hint has been received, it checks if the
#       hypothesis that correspond to that hint ID is complete and not inconsistent. In case the hypothesis is correct
#       it sends it to the robot.
#


from std_msgs.msg import String
from armor_msgs.msg import * 
from armor_msgs.srv import * 

import numpy as np
import rospy

import copy
import math
import sys
import time

import geometry_msgs.msg
from experimental_assignment1.msg import Hypo

# definition of global variables

people=[]
weapons=[]
locations=[]
hypot=[]

armor_srv=None
publisher=None

##
#   @brief main function
#	@param None
#	@return None
#
       
def main():
    global armor_srv, publisher
    rospy.init_node('hint_controller') # definition of the node
    armor_srv=rospy.ServiceProxy('armor_interface_srv', ArmorDirective) # recall for the armor service
    publisher=rospy.Publisher('/hypo', Hypo, queue_size=1000) # publisher for the /hypo topic
    rospy.wait_for_service('armor_interface_srv') 
    subscriber=rospy.Subscriber('/hint', String, check_callback) # subscriber to the /hint topic
    load_ontology()
    rospy.spin()


##
#	@brief the following function load the file which contains the ontology
#	@param None
#	@return None 
#

def load_ontology():
    try:
        req=ArmorDirectiveReq()
        req.client_name='tutorial'
        req.reference_name='ontoTest'
        req.command='LOAD'
        req.primary_command_spec='FILE'
        req.secondary_command_spec=''
        req.args=['/root/ros_ws/src/experimental_assignment1/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
        msg=armor_srv(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)


##
#	@brief function recalled when new data are available on the /hint topic
#	@param msg, from the /hint topic
#	@return None 
#

def check_callback(msg):
    global hypot
    done=0
    s=str(msg.data)
    print('New hint found '+s)
    rcv_hint=s.split('/')
    rospy.set_param('ID', rcv_hint[0])
    found=check_rcv(rcv_hint)
    
    if found==0:
        add_instance(rcv_hint[2],rcv_hint[1])
    
    check_in_ontology(rcv_hint[0], rcv_hint[1], rcv_hint[2])
    sent=check_complete_consistent(rcv_hint[0])
    
    if sent==1:
        for i in range(len(hypot)):
            if rcv_hint[0]==hypot[i]:
                done=1
                print('Information already checked\n')
        if done==0:
            print('Hint sent')
            hypot.append(rcv_hint[0])
            message=Hypo()
            message.ID=rcv_hint[0]
            temp=look_hypo(rcv_hint[0], 'who')
            message.who=temp[0]
            temp=look_hypo(rcv_hint[0], 'what')
            message.what=temp[0]
            temp=look_hypo(rcv_hint[0], 'where')
            message.where=temp[0]
            print(message)
            publisher.publish(message)
    else:
        print('Hypothesis not complete')
    
##
#	@brief function that add an instance to the loaded ontology  
#	@param name is the instance name
#	@param class_type represent the type of the instance among who, what and where
#	@return None 
# 

def add_instance(name, class_type):
    try:
        class_id=find_type(class_type)
        req=ArmorDirectiveReq()
        req.client_name='tutorial'
        req.reference_name='ontoTest'
        req.command='ADD'
        req.primary_command_spec='IND'
        req.secondary_command_spec='CLASS'
        req.args= [name, class_id]
        msg=armor_srv(req)
        res=msg.armor_response
        reason()
        disjoint(class_id)
        reason()
    except rospy.ServiceException as e:
        print(e)

##
#	@brief function that provides the type of a specific class
#	@param class_type
#	@return class name
#	

def find_type(class_type):
    if class_type=='who':
        return 'PERSON'
    if class_type=='what':
        return 'WEAPON'
    if class_type=='where':
        return 'LOCATION'

##
#	@brief function that updates the loaded ontology
#	@param None
#	@return None 
#    

def reason():
    try:
        req=ArmorDirectiveReq()
        req.client_name='tutorial'
        req.reference_name='ontoTest'
        req.command='REASON'
        req.primary_command_spec=''
        req.secondary_command_spec=''
        req.args=[]
        msg=armor_srv(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)


##
#   @brief function that disjoints all the elements that belongs to a class
#	@param class type of element that we want to disjoint
#	@return None
#	

def disjoint(class_type):
    try:
        req=ArmorDirectiveReq()
        req.client_name='tutorial'
        req.reference_name='ontoTest'
        req.command='DISJOINT'
        req.primary_command_spec='IND'
        req.secondary_command_spec='CLASS'
        req.args=[class_type]
        msg=armor_srv(req)		 
    except rospy.ServiceException as e:
        print(e)


##
#	@brief function that cleans the query related to the ontology
#	@param query, list of strings that we want clean
#	@return query after clean 
#	
#   This function, for each element of the list passed as input it splits the strings at the char '#' and takes only what is after.
#   Then it removes the last element of the remaing string.
#

def clean_queries(query):
    for i in range(len(query)):
        temp=query[i]
        temp=temp.split('#')
        index=len(temp)
        temp=temp[index-1]
        query[i]=temp[:-1]
    return query


##
#	@brief function that check if the received hint is already present in the ontology
#	@param data, list that we want to check if already received
#	@return found 
#

def check_rcv(data):
    global people, weapons, locations
    i=0
    j=0
    k=0
    found=0
    if data[1]=='who':
        for i in range(len(people)):
            if people[i]==data[2]:
                found=1
        if found==0:
            people.append(data[2])
    if data[1]=='what':
        for j in range(len(weapons)):
            if weapons[j]==data[2]:
                found=1
        if found==0:
            weapons.append(data[2])
    if data[1]=='where':
        for k in range(len(locations)):
            if locations[k]==data[2]:
                found=1
        if found==0:
            locations.append(data[2])
    return found


##
#	@brief function that adds an hypothesis to the ontology 
#	@param ID, id of the hypothesis that we want to add
#	@param class_type, type of information that we want add
#	@param name, name of information that we want add
#	@return None 
#

def add_hypo(ID,class_type,name):
    try:
        req=ArmorDirectiveReq()
        req.client_name='tutorial'
        req.reference_name='ontoTest'
        req.command='ADD'
        req.primary_command_spec='OBJECTPROP'
        req.secondary_command_spec='IND'
        req.args=[class_type,ID,name]
        msg=armor_srv(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)


##
#	@brief function that retrieves the hypothesis from its ID
#	@param ID, id of the hypothesis that we want to check
#	@param class_type, type of information that we want to retreieve
#	@return res_fin, name of entity retrieved 
#	

def look_hypo(ID,class_type):
    try:
        req=ArmorDirectiveReq()
        req.client_name='tutorial'
        req.reference_name='ontoTest'
        req.command='QUERY'
        req.primary_command_spec='OBJECTPROP'
        req.secondary_command_spec='IND'
        req.args=[class_type,ID]
        msg=armor_srv(req)
        res=msg.armor_response.queried_objects
        res_fin=clean_queries(res)
        return res_fin
    except rospy.ServiceException as e:
        print(e)


##
#	@brief function that check if the received hint is already saved in the ontology
#	@param ID, id of the hypothesis that we want to check
#	@param class_type, type of information that we want to check
#	@return None 
#

def check_in_ontology(ID,class_type,name):
    try:
        temp_hint=[]
        res_hint=look_hypo(ID, class_type)
        temp_hint.append(name)
        if res_hint != temp_hint:
            add_hypo(ID,class_type,name)
            reason()
    except rospy.ServiceException as e:
        print(e)      


##
#	@brief function which establishes whether a hypothesis is complete but inconsistent
#	@param ID, hypothesis identifier
#	@return 1 (if the hypothesis is complete and consistent) or 0 (if not)
#

def check_complete_consistent(ID):
    try:
        completed=0
        inconsistent=0
        req=ArmorDirectiveReq()
        req.client_name='tutorial'
        req.reference_name='ontoTest'
        req.command='QUERY'
        req.primary_command_spec='IND'
        req.secondary_command_spec='CLASS'
        req.args=['COMPLETED']
        msg=armor_srv(req)
        res=msg.armor_response.queried_objects
        res_hint=clean_queries(res)
        for i in range(len(res_hint)):
            if res_hint[i]==ID:
                completed=1
        req=ArmorDirectiveReq()
        req.client_name='tutorial'
        req.reference_name='ontoTest'
        req.command='QUERY'
        req.primary_command_spec='IND'
        req.secondary_command_spec='CLASS'
        req.args=['INCONSISTENT']
        msg=armor_srv(req)
        res=msg.armor_response.queried_objects
        res_hint=clean_queries(res)
        for i in range(len(res_hint)):
            if res_hint[i]==ID:
                inconsistent=1
        if completed==1 and inconsistent==0:
            return 1
        else :
            return 0
    except rospy.ServiceException as e:
        print(e)


if __name__ == '__main__':
  main()   
