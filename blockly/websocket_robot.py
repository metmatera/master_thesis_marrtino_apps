# http://www.html.it/pag/53419/websocket-server-con-python/
# pip install tornado

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

# Websocket server handler

class MyWebSocketServer(tornado.websocket.WebSocketHandler):

    def open(self):
        global websocket_server
        websocket_server = self
        print('New connection')
       
    def on_message(self, message):
        global code
        print('Code received:\n %s' % message)
        t = Thread(target=run_code, args=(message,))
        t.start()
  
    def on_close(self):
        print('Connection closed')
  
    def on_ping(self, data):
        print('ping received: %s' %(data))
  
    def on_pong(self, data):
        print('pong received: %s' %(data))
  
    def check_origin(self, origin):
        print("-- Request from %s" %(origin))
        return True



# Main loop (asynchrounous thread)

def main_loop(data):
    global run, websocket_server
    while (run):
        time.sleep(5)
        if (run and not websocket_server is None):
            print("++ping")
            websocket_server.write_message("ping")
    print("Main loop quit.")



def run_code(code):
    if (code is None):
        return
    print("=== Start code run ===")
    try:
        exec(code)
    except Exception as e:
        print("CODE EXECUTION ERROR")
        print e
    print("=== End code run ===")


# Main program
  
if __name__ == "__main__":

    # Run main thread
    #t = Thread(target=main_loop, args=(None,))
    #t.start()


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
    if (not websocket_server is None):
        websocket_server.close()
    print("Web server quit.")
    run = False    
    print("Waiting for main loop to quit...")


