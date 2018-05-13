# http://www.html.it/pag/53419/websocket-server-con-python/
# sudo -H easy_install tornado

import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web
import socket
import time
from threading import Thread

import sys
sys.path.append('../program')
sys.path.append('../scripts')

from check import *

#import robot_cmd_ros
#from robot_cmd_ros import *

#robot_cmd_ros.use_robot = False

# Global variables

websocket_server = None     # websocket handler
run = True                  # main_loop run flag
server_port = 9500          # web server port
status = "Idle"             # robot status sent to websocket

# Websocket server handler

class MyWebSocketServer(tornado.websocket.WebSocketHandler):

    def open(self):
        global websocket_server, run
        websocket_server = self
        print('New connection')
       
    def on_message(self, message):
        global code, status
        print('Received: %s' %message)
        if (message=='check'):
            status = 'Check ...'
            r = check_ROS()
            self.write_message('RESULT ros '+str(r))
            r = check_robot()
            self.write_message('RESULT robot '+str(r))
            r = check_simrobot()
            self.write_message('RESULT simrobot '+str(r))
            r = check_odom()
            self.write_message('RESULT odom '+str(r))
            r = check_laser()
            self.write_message('RESULT laser '+str(r))
            r = check_rgb_camera()
            self.write_message('RESULT rgb '+str(r))
            r = check_depth_camera()
            self.write_message('RESULT depth '+str(r))

            status = 'Idle'
        if (message=='stop'):
            print('Stop code and robot')
        elif (message[0:5]=='event'):
            print('Received event %s' %message)
            v = message.split('_')
            if (v>1):
                #set_global_param('event',v[1])
                print('Global param %s = %s' %('event',v[1]))
        else:
            print('Code received:\n%s' %message)
            if (status=='Idle'):
                t = Thread(target=run_code, args=(message,))
                t.start()
            else:
                print('Program running. This code is discarded.')

  
    def on_close(self):
        print('Connection closed')
  
    def on_ping(self, data):
        print('ping received: %s' %(data))
  
    def on_pong(self, data):
        print('pong received: %s' %(data))
  
    def check_origin(self, origin):
        #print("-- Request from %s" %(origin))
        return True



# Main loop (asynchrounous thread)

def main_loop(data):
    global run, websocket_server, status
    while (run):
        time.sleep(1)
        if (run and not websocket_server is None):
            try:
                websocket_server.write_message("STATUS "+status)
                #print(status)
            except tornado.websocket.WebSocketClosedError:
                #print('Connection closed.')
                websocket_server = None
    print("Main loop quit.")


def run_code(code):
    global status
    if (code is None):
        return
           




# Main program
  
if __name__ == "__main__":

    # Run main thread
    t = Thread(target=main_loop, args=(None,))
    t.start()

    # Run web server
    application = tornado.web.Application([
        (r'/websocketserver', MyWebSocketServer),])  
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(server_port)
    print("Bringup Websocket server listening on port %d" %(server_port))
    try:
        tornado.ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:
        print(" -- Keyboard interrupt --")

    if (not websocket_server is None):
        websocket_server.close()
    print("Bringup Websocket server quit.")
    run = False    
    print("Waiting for main loop to quit...")


