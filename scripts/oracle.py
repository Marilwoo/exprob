#! /usr/bin/env python

## @package exprob
#
# \file oracle.py
# \brief This node is the oracle. It gives hint and checks the winning hypothesis
#
# \author Maria Luisa Aiachini
# \version 1.0
# \date 02/11/2021
# 
# \details
#
#  Publishes to: <BR>
#  None
#
#  Subscribes to: <BR>
#  None
#
#  Service: <BR>
#  hint_serv
#  win
#
#  Client: <BR>
#  None
#
#  Description: <BR>
#  This node is the one that manages the hints that the robot asks.
#  When the robot asks for a hint using the request of the service of type hint,
#  the oracle answers by sending a random hint. For doing that it takes randomly
#  one of the possible hypothesis ID and then choses randomly one hint inside oif the
#  hypothesis. Then, for being sure that the same hint is not given more that once the 
#  chosen hint is removed form the list. So, finally it sends back to the robot the hint as
#  a string and the ID of the hypothesis as an int. This node also checks if the hypothesis
#  is the winning one. The robot sends the hypothesis ID throug the service of type win and
#  the oracle compares it with the winning ID. The oracle then sends a booleand back: True if
#  they match and False if not.
#


import rospy
import math
import time
import random
from exprob.srv import hint_serv, hint_servResponse, win
from exprob.msg import onto

ID1 = ["person:Rev.Green", "location:Lounge", "weapon:Spanner"]
ID2 = ["person:Prof.Plum", "location:Kitchen", "person:Mrs.Peacock"]
ID3 = ["person:Mrs.White", "location:Library", "weapon:Rope"]
ID4 = ["location:Lounge", "location:Hall", "weapon:Dagger"]
count = 4
ID = 0	
ID_list = [1, 2, 3, 4]
winner = "ID3"
result = False
pub = None

##
# \brief This function selects the random hint to be sent to the robot.
#  It first selects a random hypotehsis and thena random hint in it. When the hint is selected
#  and sent it is removed from the list to be sure it is not used more than once.
#  \param req: the request handler. Even if this server has no public "request" field the service still
#  needs it to handle the request-reply
#  \return resp: the response of the server: it contains the int for the hypothesis ID and the
#  string for the hint.


def random_hint(req):
	global ID, ID_list
	resp = hint_servResponse()
	ID = random.choice(ID_list)
	if ID == 1:
		hint_num = random.randint(0,(len(ID1)-1))
		hint_gen = ID1[hint_num]
		ID1_given = ID1.pop(hint_num)
		if ID1 == []:
			ID_list.remove(1)
		resp.ID = "ID1"
		resp.hint = hint_gen
		
	elif ID == 2:
		hint_num = random.randint(0,(len(ID2)-1))
		hint_gen = ID2[hint_num]
		ID2_given = ID2.pop(hint_num)
		if ID2 == []:
			ID_list.remove(2)
		resp.ID = "ID2"
		resp.hint = hint_gen
	elif ID == 3:
		hint_num = random.randint(0,(len(ID3)-1))
		hint_gen = ID3[hint_num]
		ID3_given = ID3.pop(hint_num)
		if ID3 == []:
			ID_list.remove(3)
		resp.ID = "ID3"
		resp.hint = hint_gen
	elif ID == 4:
		hint_num = random.randint(0,(len(ID4)-1))
		hint_gen = ID4[hint_num]
		ID4_given = ID4.pop(hint_num)
		if ID4 == []:
			ID_list.remove(4)
		resp.ID = "ID4"
		resp.hint = hint_gen                   
	return resp
	
##
#  \brief The function for checking the winning hypothesis. It takes the
#  ID of the to-be-checked hypothesis as a request, compares it with the ID
#  of the winning one and return the result of this comparison as the response
#  of the service of type win.
#  \param req: request of the service win, contains the ID of the hypothesis
#  \return result: boolean result of the comparison. True if the winning hypothesis
#  False if not.
#
def win_check(req):
	global winner, result
	if req.ID == winner:
		result = True
	else:
		result = False
	return result
	
##
#  \brief Main function of the node. It initializes the node and thetwo services:
#  the one for selecting and sending the hint: hint_serv, and the one for checking
#  the winning hypothesis.
#  \param None
#  \return None	
def main():
	global pub
	rospy.init_node('oracle')
	serv = rospy.Service('hint', hint_serv, random_hint)
	serv2 = rospy.Service('winning', win, win_check)
	
	rospy.spin()

if __name__ == '__main__':
    main()
    
