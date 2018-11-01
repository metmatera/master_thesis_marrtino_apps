# http://www.html.it/pag/53419/websocket-server-con-python/
# sudo -H easy_install tornado

import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web
import socket
import time
import os

from threading import Thread

import sys
sys.path.append('../program')
sys.path.append('../scripts')
sys.path.append('../bringup')

from tmuxsend import TmuxSend


# Global variables

websocket_server = None     # websocket handler
run = True                  # main_loop run flag
server_name = 'Config'      # server name
server_port = 9510          # config web server port
status = "Idle"             # robot status sent to websocket


# check connection
# nmcli c show --active

# Websocket server handler

class MyWebSocketServer(tornado.websocket.WebSocketHandler):

    def checkStatus(self):
        global status
        status = 'Check ...'
        self.write_message('VALUE marrtino_version %r' %os.getenv('MARRTINO_VERSION'))

        status = 'Idle'

    def open(self):
        global websocket_server, run
        websocket_server = self
        print('New connection')
        self.tmux = TmuxSend('config',['robot','network','apps'])
        self.checkStatus()

    def on_message(self, message):
        global code, status
        print('Received: %s' %message)

        if (message=='update'):
            print('TODO update')

        elif (message=='shutdown'):
            self.tmux.quitall()
            self.checkStatus()
            self.tmux.cmd(0,'sudo shutdown -h now')

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
                #print('-- WebSocketClosedError --')
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
    print("%s Websocket server listening on port %d" %(server_name,server_port))
    try:
        tornado.ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:
        print("-- Keyboard interrupt --")

    if (not websocket_server is None):
        websocket_server.close()
    print("%s Websocket server quit." %server_name)
    run = False    
    print("Waiting for main loop to quit...")


