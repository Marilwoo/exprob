#! /usr/bin/env python

## @package exprob
#
# \file onto_interface.py
# \brief This node is the interface between the robot controller node and the armor client
#
# \author Maria Luisa Aiachini
# \version 1.0
# \date 02/11/2021
# 
# \details
#
#  Publishes to: <BR>
#   /check
#
#  Subscribes to: <BR>
#  /ontology
#
#  Service: <BR>
#  None
#
#  Client: <BR>
#  Armor_interface_srv
#
#  Description: <BR>
#  This node is the node that interfaces with armor. For doing that it is a client
#  on "Armor_interface_srv". This node is used for loading the ontology. Also,
#  this node receives all the hints the robot controller receives from the Oracle
#  as a subscriber to the topic /ontology and loads them into the ontology. When 
#  the robot has a full hypothesis this node is called for chechking if the hypothesis
#  is complete and consistent, it receives the hypothesis to be checked with the topic
#  /ontology and using the publish on /check it gives the feedback to the robot controller.
#




import rospy
import math
import time
from armor_msgs.msg import * 
from armor_msgs.srv import *
from exprob.msg import onto, check_msg

ans = 0

##
# \brief This function loads the ontology file. Here is also defined the path of ontology file.
# \param: None
# \return None
#

def load_onto():
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'LOAD'
        req.primary_command_spec= 'FILE'
        req.secondary_command_spec= ''
        req.args= ['/root/ros_ws/src/exprob/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
        msg = armor_service(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)
        
        
##
#  \brief This function is used to call the different load functions depending
#  on the type of class to be loaded (person, location, weapon). Also the last
#  call is used to check the full hypothesis.
#  \param resp: it is what the node reads on the topic /ontology. It contains the ID
#  of the hint (or full hypothesis), the class of the hint an int that is used for
#  devciding what kind of operation is to be done.
#  \return None
#
def to_do(resp):
	global pub
	to_do = resp.to_do
	ID = resp.ID
	class_1 = resp.class_1
	#class_2 = resp.class_2
	#class_3 = resp.class_3
	if to_do == 1:
		load_person(ID, class_1)
	elif to_do == 2:
		load_weapon(ID, class_1)
	elif to_do == 3:
		load_location(ID, class_1)
	elif to_do == 4:
		check = check_hypothesis(ID)
		pub.publish(check)
	
##
#  \brief This function is used to load a hint of type person
#  \param ID: this parameter is the ID of the person to be loaded
#  \param class_1 this parameter contains the name of the hint
#  \return None
# 
def load_person(ID, class_1):
	global check
	try:
        	class_id="PERSON"
        	req=ArmorDirectiveReq()
        	req.client_name= 'tutorial'
        	req.reference_name= 'ontoTest'
        	req.command= 'ADD'
        	req.primary_command_spec= 'IND'
        	req.secondary_command_spec= 'CLASS'
        	req.args= [class_1, class_id]
        	msg = armor_service(req)
        	res=msg.armor_response
        	reason()
        	disjoint(class_id)
        	reason()
        	add_hypothesis(ID, "who", class_1)
        	save()
        	check = check_hypothesis(ID)
	except rospy.ServiceException as e:
		print(e)
		
##
#  \brief This function is used to load a hint of type weapon
#  \param ID: this parameter is the ID of the weapon to be loaded
#  \param class_1 this parameter contains the name of the hint
#  \return None
# 	
def load_weapon(ID, class_1):
	try:
        	class_id="WEAPON"
        	req=ArmorDirectiveReq()
        	req.client_name= 'tutorial'
        	req.reference_name= 'ontoTest'
        	req.command= 'ADD'
        	req.primary_command_spec= 'IND'
        	req.secondary_command_spec= 'CLASS'
        	req.args= [class_1, class_id]
        	msg = armor_service(req)
        	res=msg.armor_response
        	reason()
        	disjoint(class_id)
        	reason()
        	add_hypothesis(ID, "what", class_1)
        	save()
        	check = check_hypothesis(ID)
	except rospy.ServiceException as e:
		print(e)
		
##
#  \brief This function is used to load a hint of type location
#  \param ID: this parameter is the ID of the location to be loaded
#  \param class_1 this parameter contains the name of the hint
#  \return None
# 		
def load_location(ID, class_1):
	try:
        	class_id="PLACE"
        	req=ArmorDirectiveReq()
        	req.client_name= 'tutorial'
        	req.reference_name= 'ontoTest'
        	req.command= 'ADD'
        	req.primary_command_spec= 'IND'
        	req.secondary_command_spec= 'CLASS'
        	req.args= [class_1, class_id]
        	msg = armor_service(req)
        	res=msg.armor_response
        	reason()
        	disjoint(class_id)
        	reason()
        	add_hypothesis(ID, "where", class_1)
        	save()
        	check = check_hypothesis(ID)
	except rospy.ServiceException as e:
		print(e)

##
#  \brief This function is used to save the ontology
#  \param: None
#  \return None
#
def save():
	try:
		req=ArmorDirectiveReq()
		req.client_name= 'tutorial'
		req.reference_name= 'ontoTest'
		req.command= 'SAVE'
		req.primary_command_spec= 'INFERENCE'
		req.args= ["/root/ros_ws/src/exprob/cluedo_ontology.owl"]
		msg = armor_service(req)
		res=msg.armor_response
	except rospy.ServiceException as e:
		print(e)
		
##
#  \brief This function runs the reasoner
#  \param: None
#  \return None
#	
def reason():
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'REASON'
        req.primary_command_spec= ''
        req.secondary_command_spec= ''
        req.args= []
        msg = armor_service(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)
	
##
#  \brief This function updates the ontology
#  \params class_id: this parameter is the type: PLACE, WEAPON, PERSON
#  \return None 
	
def disjoint(class_id):
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'DISJOINT'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [class_id]
        msg = armor_service(req)		 
    except rospy.ServiceException as e:
        print(e)

##
#  \brief This function is used to add an hint to an hypothesis in the ontology
#  \param ID: this parameter is the ID of the hypothesis
#  \param class_type: this is the class type of the hint to be added to the hypothesis
#  \param name: os the name of the hint to be added to the hypothesis   
#  \return None   
#
def add_hypothesis(ID, class_type, name):
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'OBJECTPROP'
        req.secondary_command_spec= 'IND'
        req.args= [class_type, ID, name]
        msg = armor_service(req)
        res=msg.armor_response
        save()
        check = check_hypothesis(ID)
    except rospy.ServiceException as e:
        print(e)

##
#  \brief This function cleans the query returned from the ontology
#  \param query: the list of strings to be cleaned
#  \return the cleaned query
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
#  \brief This function is used to check the hypothesis. Here the armor server is called
#  two times: one for checking the completeness of the hypothesis and the second one to
#  check the consistency
#  \param ID: The ID of the hypothesis to be checked
#  \return returns True if the hypothesis is complete and consistent
#  it returns False if not
#  

def check_hypothesis(ID):
    try:
        completed=0
        inconsistent=0
        reason()
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['COMPLETED']
        msg = armor_service(req)
        res=msg.armor_response.queried_objects
        res_final=clean_queries(res)
        for i in range(len(res_final)):
            if res_final[i]==ID:
                completed=1
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['INCONSISTENT']
        msg = armor_service(req)
        res=msg.armor_response.queried_objects
        res_final=clean_queries(res)
        for i in range(len(res_final)):
            if res_final[i]==ID:
                inconsistent=1
        if completed==1 and inconsistent==0:
            return True
        else :
            return False
    except rospy.ServiceException as e:
        print(e)
        
##
# \brief This is the mai function of the node. When the node is initialized it initialized
#  all the publishers, subscribers and client. Also it loads the ontology file.       
# \param: None
# \return: None
#
        
def main():
	global  armor_service, pub
	rospy.init_node('onto_interface')
	armor_service = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
	rospy.Subscriber("/ontology", onto, to_do)
	rospy.wait_for_service('armor_interface_srv')
	pub = rospy.Publisher("/check", check_msg, queue_size = 10)
	load_onto()
	rospy.spin()
        
if __name__ == '__main__':
    main()
    
