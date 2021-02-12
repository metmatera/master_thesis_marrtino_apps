#!/usr/bin/env python

from __future__ import print_function

import thread
import socket

import argparse

import sys, time, os, glob, shutil, math, datetime

from tmuxsend import TmuxSend


def run_server():
    port = 9240

    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #sock.settimeout(3)
    # Bind the socket to the port
    server_address = ('', port)
    sock.bind(server_address)
    sock.listen(1)
    print("ROS navigation server started on port %d ..." %port)

    tmux = TmuxSend('bringup', ['loc','nav','cmd'])

    connected = False
    dorun = True
    while dorun:

        if not connected:
            print("-- Waiting for connection ...")
        while (dorun and not connected):
            try:
                # Wait for a connection
                connection, client_address = sock.accept()
                connected = True
                print ('-- Connection from %s'  %client_address[0])
            except KeyboardInterrupt:
                print("User interrupt (quit)")
                dorun = False
            except Exception as e:
                print(e)
                pass # keep listening
    
        if not dorun:
            return

        # print("-- Waiting for data...")
        data = None
        while dorun and connected and data is None:
            # receive data
            try:
                #connection.settimeout(3) # timeout when listening (exit with CTRL+C)
                data = connection.recv(320)  # blocking
                data = data.strip()
            except KeyboardInterrupt:
                print("User interrupt (quit)")
                dorun = False
            except socket.timeout:
                data = None
                print("socket timeout")

        if data is not None:
            if len(data)==0:
                connected = False
            else:
                print(data)
                navfolder = "~/src/marrtino_apps/navigation"
                if data=='@amcl':
                    tmux.cmd(0,'cd %s' %navfolder)
                    tmux.cmd(0,'python startloc.py')
                elif data=='@movebase':
                    tmux.cmd(1,'cd %s' %navfolder)
                    tmux.cmd(1,'roslaunch move_base.launch')
                else:
                    print('Unknown command %s')




if __name__ == '__main__':
    run_server()

