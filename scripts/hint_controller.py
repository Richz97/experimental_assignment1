#! usr/bin/env python2

# The program elaborates the hints received

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

# main

def main():
    global armor_srv, publisher
    rospy.init_node('hint_controller')
    armor_srv=rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    publisher=rospy.Publisher('/hypo', Hypo, queue_size=1000)
    rospy.wait_for_service('armor_interface_srv')
    subscriber=rospy.Subscriber('/hint', String, check_callback)
    load_ontology()
    rospy.spin()


# the following function load the file which contains the ontology

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

# function recalled when new data are available on the /hint topic

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
    
# function that add an instance to the loaded ontology

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

# function that provides the type of a specific class

def find_type(class_type):
    if class_type=='who':
        return 'PERSON'
    if class_type=='what':
        return 'WEAPON'
    if class_type=='where':
        return 'LOCATION'

# function that updates the loaded ontology

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


# function that disjoints all the elements that belongs to a class

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

# function that cleans the query related to the ontology

def clean_queries(query):
    for i in range(len(query)):
        temp=query[i]
        temp=temp.split('#')
        index=len(temp)
        temp=temp[index-1]
        query[i]=temp[:-1]
    return query

# function that check if the received hint is already present in the ontology

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

# function that adds an hypothesis to the ontology

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


# function that retrieves the hypothesis from its ID

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

# function that check if the received hint is already saved in the ontology

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

# function which establishes whether a hypothesis is complete but inconsistent

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
