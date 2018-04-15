# http://www.html.it/pag/53419/websocket-server-con-python/
# sudo -H easy_install tornado

import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web
import socket
import time
from threading import Thread

#from dummy_robot import begin,end,forward,backward,left,right

import sys
sys.path.append('../program')

from robot_cmd_ros import *

# Global variables

websocket_server = None     # websocket handler
run = True                  # main_loop run flag
server_port = 9000          # web server port
code = None
status = "Idle"             # robot status sent to websocket

# Websocket server handler

class MyWebSocketServer(tornado.websocket.WebSocketHandler):

    def open(self):
        global websocket_server, run
        websocket_server = self
        print('New connection')
       
    def on_message(self, message):
        global code, status
        if (message=='stop'):
            print('Stop code and robot')
            robot_stop_request()
        else:
            print('Code received:\n%s' % message)
            if (status=='Idle'):
                t = Thread(target=run_code, args=(message,))
                t.start()
            else:
                print('Program running. This code is discarded.')
        self.write_message('OK')
  
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
                websocket_server.write_message(status)
                #print(status)
            except tornado.websocket.WebSocketClosedError:
                #print('Connection closed.')
                websocket_server = None
    print("Main loop quit.")


def beginend(code):
    r = ""
    v = code.split("\n")
    incode = False
    for i in v:
        if (i=='begin()'):
            r = r+i+'\n'
            incode = True
        elif (i=='end()'):
            r = r+i+'\n'
            incode = False
        elif (incode):
            r = r+i+'\n'
    return r            


def run_code(code):
    global status
    if (code is None):
        return
    print("=== Start code run ===")
    #code = beginend(code)
    print("Executing")
    print(code)
    try:
        status = "Executing program"
        exec(code)
    except Exception as e:
        print("CODE EXECUTION ERROR")
        print e
    status = "Idle"
    print("=== End code run ===")


# Main program
  
if __name__ == "__main__":

    # Run main thread
    t = Thread(target=main_loop, args=(None,))
    t.start()

    # Run robot
    begin('websocket_robot')

    # Run web server
    application = tornado.web.Application([
        (r'/websocketserver', MyWebSocketServer),])  
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(server_port)
    print("Websocket server listening on port %d" %(server_port))
    try:
        tornado.ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:
        print(" -- Keyboard interrupt --")

    # Quit
    end()

    if (not websocket_server is None):
        websocket_server.close()
    print("Web server quit.")
    run = False    
    print("Waiting for main loop to quit...")


