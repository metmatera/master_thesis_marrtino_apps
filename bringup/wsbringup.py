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

import check
from check import *

from tmuxsend import TmuxSend


# Global variables

websocket_server = None     # websocket handler
run = True                  # main_loop run flag
server_name = 'Bringup'     # server name
server_port = 9500          # web server port
status = "Idle"             # robot status sent to websocket




# Websocket server handler

class MyWebSocketServer(tornado.websocket.WebSocketHandler):

    def getversion(self):
        f = open('/home/ubuntu/.marrtino_version','r')
        v = f.readline().strip()
        f.close()
        return v

    def checkStatus(self):
        global status
        status = 'Check ...'
        self.write_message('VALUE marrtino_version %r' %self.getversion())
        r = check_ROS()
        self.write_message('RESULT ros '+str(r))
        if (r):
            rospy.init_node('wsserver_check', disable_signals=True)
            self.write_message('VALUE rosnodes %r' %check.nodenames)
            self.write_message('VALUE rostopics %r' %check.topicnames)

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

        r = check_tf('map', 'odom')
        self.write_message('RESULT tf_map_odom '+str(r))
        r = check_tf('odom', 'base_frame')
        self.write_message('RESULT tf_odom_base '+str(r))
        r = check_tf('base_frame', 'laser_frame')
        self.write_message('RESULT tf_base_laser '+str(r))
        r = check_tf('base_frame', 'rgb_camera_frame')
        self.write_message('RESULT tf_base_rgb '+str(r))
        r = check_tf('base_frame', 'depth_camera_frame')
        self.write_message('RESULT tf_base_depth '+str(r))

        status = 'Idle'


    def open(self):
        global websocket_server, run
        websocket_server = self
        print('New connection')
        self.tmux = TmuxSend('bringup',['robot','laser','camera','joystick','audio','wsrobot'])
        self.checkStatus()

    def on_message(self, message):
        global code, status
        print('Received: %s' %message)

        if (message=='stop'):
            print('!!! EMERGENCY STOP !!!')

        elif (message=='check'):
            self.checkStatus()

        elif (message=='ros_quit'):
            self.tmux.quitall()
            self.checkStatus()

        elif (message=='robot_start'):
            self.tmux.roslaunch(1,'robot','robot')
            time.sleep(3)
            self.checkStatus()
        elif (message=='robot_kill'):
            self.tmux.roskill('orazio')
            self.tmux.roskill('state_pub_robot')
            time.sleep(3)
            self.tmux.killall(1)
            self.checkStatus()

        elif (message=='simrobot_start'):
            self.tmux.roslaunch(1,'stage','simrobot')
            time.sleep(3)
            self.checkStatus()
        elif (message=='simrobot_kill'):
            self.tmux.roskill('stageros')
            time.sleep(3)
            self.tmux.killall(1)
            time.sleep(3)
            self.checkStatus()

        elif (message=='usbcam_start'):
            self.tmux.roslaunch(3,'camera','usbcam')
            time.sleep(3)
            self.checkStatus()
        elif (message=='usbcam_kill'):
            self.tmux.roskill('usb_cam')
            self.tmux.roskill('state_pub_usbcam')
            time.sleep(3)
            self.tmux.killall(3)
            time.sleep(3)
            self.checkStatus()
            time.sleep(3)
            self.checkStatus()

        elif (message=='astra_start'):
            self.tmux.roslaunch(3,'camera','astra')
            time.sleep(3)
            self.checkStatus()
        elif (message=='astra_kill'):
            self.tmux.roskill('astra')
            self.tmux.roskill('state_pub_astra')
            time.sleep(3)
            self.tmux.killall(3)
            time.sleep(3)
            self.checkStatus()

        elif (message=='xtion_start'):
            self.tmux.roslaunch(3,'camera','xtion')
            time.sleep(3)
            self.checkStatus()
        elif (message=='xtion_kill'):
            self.tmux.roskill('xtion')
            self.tmux.roskill('state_pub_xtion')
            time.sleep(3)
            self.tmux.killall(3)
            time.sleep(3)
            self.checkStatus()

        elif (message=='hokuyo_start'):
            self.tmux.roslaunch(2,'laser','hokuyo')
            time.sleep(3)
            self.checkStatus()
        elif (message=='hokuyo_kill'):
            self.tmux.roskill('hokuyo')
            self.tmux.roskill('state_pub_laser')
            time.sleep(3)
            self.tmux.killall(2)
            time.sleep(3)
            self.checkStatus()

        elif (message=='rplidar_start'):
            self.tmux.roslaunch(2,'laser','rplidar')
            time.sleep(3)
            self.checkStatus()
        elif (message=='rplidar_kill'):
            self.tmux.roskill('rplidar')
            self.tmux.roskill('state_pub_laser')
            time.sleep(3)
            self.tmux.killall(2)
            time.sleep(3)
            self.checkStatus()

        elif (message=='astralaser_start'):
            self.tmux.roslaunch(2,'laser','astra_laser')
            time.sleep(3)
            self.checkStatus()
        elif (message=='astralaser_kill'):
            self.tmux.roskill('astra')
            self.tmux.roskill('depth2laser')
            self.tmux.roskill('state_pub_astra_laser')
            time.sleep(3)
            self.tmux.killall(2)
            time.sleep(3)
            self.checkStatus()

        elif (message=='xtionlaser_start'):
            self.tmux.roslaunch(2,'laser','xtion_laser')
            time.sleep(3)
            self.checkStatus()
        elif (message=='xtionlaser_kill'):
            self.tmux.roskill('xtion')
            self.tmux.roskill('depth2laser')
            self.tmux.roskill('state_pub_xtion_laser')
            time.sleep(3)
            self.tmux.killall(2)
            time.sleep(3)
            self.checkStatus()

        elif (message=='joystick_start'):
            self.tmux.roslaunch(4,'teleop','teleop')
            time.sleep(3)
            self.checkStatus()
        elif (message=='joystick_kill'):
            self.tmux.roskill('joystick')
            self.tmux.roskill('joy')
            time.sleep(3)
            self.tmux.killall(4)
            time.sleep(3)
            self.checkStatus()

        elif (message=='audio_start'):
            self.tmux.python(5,'audio','audio_server.py')
            time.sleep(3)
            self.checkStatus()
        elif (message=='audio_kill'):
            self.tmux.killall(5)
            time.sleep(3)
            self.checkStatus()

        elif (message=='wsrobot_start'):
            self.tmux.python(6,'blockly','websocket_robot.py')
            time.sleep(3)
            self.checkStatus()
        elif (message=='wsrobot_kill'):
            self.tmux.killall(6)
            time.sleep(3)
            self.checkStatus()

        elif (message=='shutdown'):
            self.tmux.quitall()
            self.checkStatus()
            self.tmux.cmd(1,'sudo shutdown -h now')

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
                # print('-- WebSocketClosedError --')
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


