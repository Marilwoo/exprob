#! /usr/bin/env python

## @package exprob
#
# \file move_to.py
# \brief This node is the move to. It simuates the robot movement.
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
#  move_to_srv
#
#  Client: <BR>
#  None
#
#  Description: <BR>
#  This node is the one that simulates the movement of the robot. It receives
#  as a integer the room in which the robot needs ot move through the request of the
#  service of type moveto. It assigns  every integer to a position and then just wait
#  to simulate the robot movement.
#

import rospy
import math
import time
import numpy as np
from exprob.srv import move_to_srv, move_to_srvResponse

##
#  \brief This is the function simulates the robot movement. It receives the target position and simulates the movement via a sleep.
#  \param room: is the value that the function receives as a request from the client moveto
#  \return resp: the return value for the service, it contains a boolean telling if the
#  robot has reached the room.
#
def move(req):
	resp = move_to_srvResponse()
	posx = req.posx
	posy = req.posy
	
	time.sleep(2)	
	resp = True
	return resp 

##
#  \brief Main function of the node. Contains the initialization of the node and
#  the one for the servie moveto.
#  \param None
#  \return None
def main():
	global pub
	rospy.init_node('move_to')
	serv = rospy.Service('moveto', move_to_srv, move)
	
	rospy.spin()

if __name__ == '__main__':
    main()
