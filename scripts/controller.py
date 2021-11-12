#! /usr/bin/env python

## @package exprob
#
# \file controller.py
# \brief This ndoe is the robot controller.
#
# \author Maria Luisa Aiachini
# \version 1.0
# \date 02/11/2021
# 
# \details
#
#  Publishes to: <BR>
#   /ontology
#
#  Subscribes to: <BR>
#  /check
#
#  Service: <BR>
#  None
#
#  Client: <BR>
#  hint
#  winning
#  moveto
#
##  Description: <BR>
#
#  This node is the robot controller node. It is the node that tells the robot where to
#  move between the rooms. For moving it is client of the service of type moveto: it sends it
#  the room in which the robot has to go and the server returns a boolean to know when the target
#  has been reached. Once the robot reaches the right room it asks the oracle for a hint
#  using the client of type hint_serv, which returns the ID and the string containig the hint.
#  Everytime a hint is received the robot sends it to the ontology interface by publishing on
#  the toipic /ontology. This goes on untill one hypothesis is full. When this happens the
#  controller sends the hypothesis to be checked to the ontology interface using the
#  publisher /ontology in order to check if it is complete and consistent. The result of this check
#  is returned to the controller by being a subscriber to the topi /check.
#  If the hypothesis is complete and consistent the controller goes on
#  and goes to home to check if the hypothesis is the winning one. For doing that it sends the
#  Id of the hypothesis to the oracle using the service of type winning. If the hypothesis is the
#  right one the program ends, if not the robot goes on to the rooms searching for other hints
#  and hypothesis.
#


import rospy
import math
import time
import random
import numpy as np
from exprob.srv import hint_serv, hint_servResponse, win, move_to_srv
from exprob.msg import onto, check_msg

actual_position = 0
next_position = 0
state = 1
cond = True
count1 = 0
count2 = 0
count3 = 0
count4 = 0
ID1 = []
ID2 = []
ID3 = []
ID4 = []
check = None
places = ["Kitchen","Ballroom","Dining Room","Billiard Room","Library","Lounge","Hall","Study"]
##
#  \brief this function is the one that decides randomly the "next room" the robot
#  has to go to.
#  \param None
#  \return None
#
def next_room():
	global next_position, state, places
	time.sleep(1)
	while next_position == actual_position:
		next_position = random.randint(0, 7)	
	print("Next room: %s" %places[next_position])
	print("Going")
	state = 2
	time.sleep(2)
##
#  \brief This function is the one that simulate the movement of the robot, it calls the client
#  for reaching the next position giving it the position of the room that has to be reached.
#  \param None
#  \return None
#
def go_to_room():
	global state, cli3, next_position, actual_position
	if next_position == 0:
		posx = 1
		posy = 1
	elif next_position == 1:
		posx = 1
		posy = 2
	elif next_position == 2:
		posx = 1
		posy = 3
	elif next_position == 3:
		posx = 2
		posy = 1
	elif next_position == 4:
		posx = 2
		posy = 3
	elif next_position == 5:
		posx = 3
		posy = 1
	elif next_position == 6:
		posx = 3
		posy = 2
	elif next_position == 7:
		posx = 3
		posy = 3

	resp = cli3(posx, posy).reached
	if resp == True:
		print("Room reached!")
		actual_position = next_position
		state = 3
	else:
		print("Movement Error")
##
#  \brief This function is the one that asks the oracle for a hint. It calls the
#  service hint_serv and takes the hint. It stores the hint in the right array and sends it 
#  to the ontology for it ro be added as an instance and as a part of an hypothesis using the 
#  publisher to the topic /ontology. If one of the hypothesis is of the right lenght it sends
#  it to the function check_hypo().
#  \param None
#  \return None
#
def ask_hint():
	global state, cli, new_hint, new_ID, new_type, ID1, ID2, ID3, ID4, pub, check, count1, count2, count3, count4, state
	print("Asking for hint")
	new_hint = []
	resp = cli()
	msg = onto()
	new_hint_complete = resp.hint
	new_hint_complete = new_hint_complete.split(":")
	new_hint.append(new_hint_complete[0])
	new_hint.append(new_hint_complete[1])
	new_ID = resp.ID
	msg.class_1 = new_hint[1]
	if new_hint[0] == "person":
		msg.to_do = 1
	elif new_hint[0] == "weapon":
		msg.to_do = 2
	elif new_hint[0] == "location":
		msg.to_do = 3
	msg.ID = new_ID
	pub.publish(msg)
	time.sleep(1)
	print("Hint received: %s" %new_hint)
	time.sleep(1)
	print()	
	if new_ID == "ID1":
		ID1.append(new_hint[0])
		ID1.append(new_hint[1])
		count1 = count1 + 1
		if count1 == 3:
			check_hypo(new_ID, ID1)
		else:
			print("Goint to another room")
			time.sleep(1)
			state = 1
			
	elif new_ID == "ID2":
		ID2.append(new_hint[0])
		ID2.append(new_hint[1])
		count2 = count2 + 1
		if count2 == 3:
			check_hypo(new_ID, ID2)
		else:
			time.sleep(1)
			print("Goint to another room")
			time.sleep(1)
			state = 1
			
	elif new_ID == "ID3":
		ID3.append(new_hint[0])
		ID3.append(new_hint[1])
		count3 = count3 + 1
		if count3 == 3:
			check_hypo(new_ID, ID3)
		else:
			print("Goint to another room")
			time.sleep(1)
			state = 1
			
	elif new_ID == "ID4":
		ID4.append(new_hint[0])
		ID4.append(new_hint[1])
		count4= count4 + 1
		if count4 == 3:
			check_hypo(new_ID, ID4)
		else:
			print("Goint to another room")
			time.sleep(1)
			state = 1		
			
##
#  \brief This function is the one that sends the hypothesis to the ontology for it to
#  check if it is complete and consistent. It sends it using the topic /ontology and receives
#  the feedback by the topic /check. If the check result is True then the robot goes to the
#  function go_to_home to check if the hypothesis is the winning one.
#  \param ID: the id of the hypothesis that needs to be checked
#  \param ID_list: the list of hints contained in the hypothesis
#  \return None
#
def check_hypo(ID, ID_list):
	global pub, state, hypo
	msg = onto()
	msg.to_do = 4
	msg.ID = ID
	time.sleep(3)
	print("I have a full hypothesis: %s" %ID_list)
	hypo = ID_list
	msg.class_1 = ""
	pub.publish(msg)
	time.sleep(5)
	if check == True:
		print("Hypotesys consistent, let's see if it is the right one")
		state = 4
	else:
		print("hypothesis not consistent or complete, fiding another one")
		ID1 = []
		state = 1
##
#  \brief This function is for making the robot go to home to check the hpothesys.
#  It sends the ID of the hypothesis to the oracle using the service of type win
#  and returns the feedback. If the hypothesis is the winning one the program exits,
#  if the hypothesis is not the correct one the robot starts again going aroung between
#  the rooms to get more hints and more hypothesis.
#  \param None
#  \return None
#
def go_to_home():
	global cli2, cli3, state, hypo
	w = hypo.index("weapon")
	l = hypo.index("location")
	p = hypo.index("person")
	print("Hypothesis: It was %s with the %s in the %s" %(hypo[p+1], hypo[w+1], hypo[l+1]))
	print("\nGoint to check if the hypothesis is correct!")
	posx = 0
	posy = 0
	resp3 = cli3(posx, posy).reached
	if resp3 == True:
		resp2 = cli2(new_ID)
		if resp2.result == True:
			print("hypothesis correct!\nI won!!")
			exit()
		else:
			print("Not the winning hypothesis, trying again")
			state = 1
	else:
		print("Movement Error")
##
#  \brief This funtion is the callback for the subscriber on the topic
#  check. It stores the result of the check of the winning hypothesis.
#  \param ans: the value retrieved by the subscriber
#  \return None
#		
def hint_check(ans):
	global check
	check = ans.check

##
#  \brief The main of this node. It initializes the node all the publishers, subscribers and  clients.
#  There are the client /hint for receiving the hint, the winning for checking if it is the winning
#  hypothesis and the move to for moving to the room. There are the subscriber for check for checking
#  if the hypothesis is complete and consistent, the ontology for sending the hints to the ontlogy.
#  \param None
#  \return None
#
def main():
	global cli, state, cli2, cli3, pub, check, sub
	rospy.init_node('controller')
	cli = rospy.ServiceProxy('hint', hint_serv)
	cli2 = rospy.ServiceProxy('winning', win)
	cli3 = rospy.ServiceProxy('moveto', move_to_srv)
	sub = rospy.Subscriber("check", check_msg, hint_check)
	pub = rospy.Publisher("/ontology", onto, queue_size = 10)
	rospy.wait_for_service('armor_interface_srv')
		
	while (cond == True):
		if state == 1:
			next_room()	  
		elif state == 2:
			go_to_room()
		elif state == 3:
			ask_hint()
		elif state == 4:
			go_to_home()
		
	while not rospy.is_shutdown():
		rate.sleep()
	

if __name__ == '__main__':
    main()
