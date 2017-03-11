#!/usr/bin/env python

import socket
import sys



TCP_IP = '127.0.0.1'
TCP_PORT = 5001
BUFFER_SIZE = 200

MESSAGE = "begin; hello; wait; start; wait; bip; wait; bop; end"

# Use: ./robot_program_client.py <HOST> <PORT>


if __name__ == "__main__":

	if (len(sys.argv)>1):
		TCP_IP = sys.argv[1]
		TCP_PORT = int(sys.argv[2])

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))
	s.send(MESSAGE)
	data = s.recv(BUFFER_SIZE)
	s.close()

	print "received data:", data

