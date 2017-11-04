#!/usr/bin/env python

import sys
import os
import socket
import importlib
import re
import robot_cmd

robot_cmd_mod = importlib.import_module("robot_cmd")


TCP_IP = ''
TCP_PORT = 5000
BUFFER_SIZE = 200


def exec_fn(fn,vd):
	largs = []
	for i in range(1,len(vd)):
		if (vd[i]!=''):
			largs += [ vd[i] ]
	print 'Executing ',fn,' with args ',largs
	try:
		if (len(largs)==0):
			fn()
		elif (len(largs)==1):
			fn(int(largs[0]))
		elif (len(largs)==2):
			fn(int(largs[0]),int(largs[1]))
	except:
		print "ERROR: executing",fn," with args ",largs



def exec_cmd(data):
	print "received command:", data
	vd = re.split('[(,)_]',data)
	try:
		fn = getattr(robot_cmd_mod, vd[0])
	except:
		print "ERROR: function",vd[0],"not found"
		fn = None
	if (not fn is None):
		exec_fn(fn,vd)


def start_server():
	global TCP_IP
	global TCP_PORT
	global BUFFER_SIZE

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind((TCP_IP,TCP_PORT))
	s.listen(1)

	#cmd = "play --no-show-progress --null -c2 synth sin gain -100 &"
	#os.system(cmd)

	run=True

	while run:
		print "Robot Program Server: waiting for connections port", TCP_PORT
		conn, addr = s.accept()
		print "Connection address:", addr
		connected = True
		while connected:
			try:
				data = conn.recv(BUFFER_SIZE)
			except:
				print "Connection closed."
				break
			if not data: break
			print "Received: ",data
			conn.send("OK\n")
			vdata = re.split("[\r\n;]",data)
			for i in range(0,len(vdata)):
				if (vdata[i]=="quit"):
					connected=False
					break
				else:
					com = vdata[i].strip()
					if (len(com)>0):
						exec_cmd(com)

		conn.close()
		print "Closed connection"




if __name__ == "__main__":
	if (len(sys.argv)==2):
		TCP_PORT = int(sys.argv[1])
	elif (len(sys.argv)==3):
		TCP_PORT = int(sys.argv[2])
		if (sys.argv[1]=="-norobot"):
			robot_cmd.userobot = False
	else:
		print "Use: robot_program_server [-norobot] <TCP_port>"
		sys.exit(1)

	start_server()

