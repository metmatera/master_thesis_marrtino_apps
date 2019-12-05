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


    def open(self):
        global websocket_server, run
        websocket_server = self
        print('New connection')
        self.tmux = TmuxSend('config',['robot','network','apps'])
        self.home = os.getenv('HOME')
        if (self.home=='/root'): # started at boot on MARRtino cards
            self.home = '/home/ubuntu'
        self.mahome = os.getenv('MARRTINO_APPS_HOME')
        if self.mahome==None:
            self.mahome = self.home+'/src/marrtino_apps'
        self.checkStatus()


    def setStatus(self, st):
        # do not use blank spaces in the status string
        global status
        status = st
        try:
            self.write_message("STATUS "+status)
            #print(status)
        except tornado.websocket.WebSocketClosedError:
            print('-- WebSocketClosedError --')



    def checkStatus(self):
        self.setStatus('Checking...')
        self.write_message('VALUE marrtino_version %s' %self.getMARRtinoVersion())
        self.write_message('VALUE marrtino_apps_version %s' %self.getMARRtinoAppVersion())
        self.setStatus('Idle')




    def getMARRtinoVersion(self):
        print('Checking MARRtino version from MARRTINO_VERSION env ...')
        v1 = os.getenv('MARRTINO_VERSION')
        print('Checking MARRtino version from $HOME/.marrtino_version file ...')
        try:
            f = open('%s/.marrtino_version' %self.home, 'r')
            v2 = f.readline().strip()
            f.close()
        except Exception as e:
            v2 = 'None'
            print(e)

        print('MARRtino version read %s %s' %(v1,v2))
        if (v1==None):
            return v2
        elif (v2==None):
            return v1
        elif (v1>v2):
            return v1
        else:
            return v2


    def getMARRtinoAppVersion(self):
        print('Checking MARRtino Apps version from git log ...')
        self.tmux.cmd(3,'cd %s' %self.mahome)
        self.tmux.cmd(3,'git log | head -n 4 | grep Date > /tmp/.marrtinoapp_version', blocking=True)
        time.sleep(1)
        try:
            f = open('/tmp/.marrtinoapp_version','r')
            v = f.readline().strip()
            lista = v.split(':',1)
            v = lista[1]
            lista = v.split('+')
            v=lista[0]
            f.close()
            print('    ... read %s' %v)
        except Exception as e:
            v = 'None'
            print(e)
        return v


    def on_message(self, message):
        global code, status
        print('Received: %s' %message)

        if (message=='updatesystem'):
            print('system update')
            self.setStatus('Updating...')
            self.tmux.cmd(3,'cd %s/install' %self.home)
            self.tmux.cmd(3,'python marrtino_update.py --yes', blocking=True)
            time.sleep(3)
            self.checkStatus()

        elif (message=='updatemarrtinoapps'):
            print('marrtino_apps update')
            self.setStatus('Updating...')
            self.tmux.cmd(3,'cd %s' %self.mahome)
            self.tmux.cmd(3,'git pull', blocking=True)
            time.sleep(3)
            self.checkStatus()

        elif (message=='updatemodim'):
            print('MODIM update')
            self.setStatus('Updating...')
            self.tmux.cmd(3,'cd %s/src/modim' %self.home)
            self.tmux.cmd(3,'git pull', blocking=True)
            time.sleep(3)
            self.checkStatus()

        elif (message=='updaterchome'):
            print('RCHOME-learn update')
            self.setStatus('Updating...')
            self.tmux.cmd(3,'cd %s/src/rc-home-edu-learn-ros' %self.home)
            self.tmux.cmd(3,'git pull', blocking=True)
            time.sleep(3)
            self.checkStatus()

        elif (message=='shutdown'):
            self.setStatus('Shutdown!!!')
            self.tmux.quitall()
            self.checkStatus()
            self.tmux.cmd(0,'sudo shutdown -h now')

        elif (message=='reboot'):
            self.setStatus('Reboot!!!')
            self.tmux.quitall()
            self.checkStatus()
            self.tmux.cmd(0,'sudo reboot')

        elif(message=='flash'):
            print('firmware upload')
            self.tmux.cmd(3,'cd %s/src/srrg/srrg2_orazio/srrg2_orazio/firmware_build/atmega2560/' %self.home)
            self.tmux.cmd(3,'make')
            self.tmux.cmd(3,'make orazio.hex')
            self.tmux.cmd(3,'make clean')

        elif(message=='firmwareparam'):
            print('firmware parameters upload')
            self.tmux.cmd(3,'cd %s/config' %self.mahome)
            self.tmux.cmd(3,'cat upload_config.script | rosrun srrg2_orazio orazio -serial-device /dev/orazio')

        elif (message=='startweb'):
            print('start orazio web server')
            self.tmux.cmd(1,'cd %s/config' %self.mahome)
            self.tmux.cmd(1,'source run_orazio_web.bash')

        elif (message=='quitweb'):
            print('quit orazio web server')
            self.tmux.cmd(1,'quit')

        elif (message=='wirelessAP'):
            print('quit orazio web server')
            self.tmux.cmd(3,'sudo nmcli c up MARRtinoAP')

        elif (message=='wirelessHome'):
            print('quit orazio web server')
            self.tmux.cmd(3,'sudo nmcli c up MARRtinoHome')

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


