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

# Global variables

websocket_server = None     # websocket handler
run = True                  # main_loop run flag
server_port = 9500          # web server port
status = "Idle"             # robot status sent to websocket
nwindows = 0

def tmux_init():
    global nwindows
    os.system('tmux -2 new-session -d -s bringup') # tmux session
    os.system('tmux select-window -t bringup:0')
    os.system('tmux rename-window \'quit\'')                  # window 0 - quit
    os.system('tmux new-window -t bringup:1 -n \'robot\'')    # window 1 - robot
    os.system('tmux new-window -t bringup:2 -n \'laser\'')    # window 2 - laser
    os.system('tmux new-window -t bringup:3 -n \'camera\'')   # window 3 - camera
    os.system('tmux new-window -t bringup:4 -n \'joystick\'') # window 4 - joystick
    os.system('tmux new-window -t bringup:5 -n \'audio\'')    # window 5 - audio
    os.system('tmux new-window -t bringup:6 -n \'wsrobot\'')  # window 6 - WS robot
    nwindows = 6

def tmux_launch(wid, mdir, mlaunch, mparams=''):
    os.system('tmux select-window -t bringup:%d' %(wid))
    os.system('tmux send-keys "cd $MARRTINO_APPS_HOME/%s" C-m' %(mdir))
    os.system('tmux send-keys "roslaunch %s.launch %s" C-m' %(mlaunch, mparams))

def tmux_python(wid, mdir, mpy, mparams=''):
    os.system('tmux select-window -t bringup:%d' %(wid))
    os.system('tmux send-keys "cd $MARRTINO_APPS_HOME/%s" C-m' %(mdir))
    os.system('tmux send-keys "python %s %s" C-m' %(mpy, mparams))

def tmux_kill(rosnode):
    wid = 0
    os.system('tmux select-window -t bringup:%d' %(wid))
    os.system('tmux send-keys "rosnode kill %s" C-m' %(rosnode))

def tmux_Cc(wid):
    os.system('tmux select-window -t bringup:%d' %(wid))
    os.system('tmux send-keys C-c')

def tmux_Ck(wid):
    os.system('tmux select-window -t bringup:%d' %(wid))
    os.system('tmux send-keys C-\\')


# Websocket server handler

class MyWebSocketServer(tornado.websocket.WebSocketHandler):

    def checkStatus(self):
        global status
        status = 'Check ...'
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
        self.checkStatus()


    def on_message(self, message):
        global code, status
        print('Received: %s' %message)

        if (message=='stop'):
            print('!!! EMERGENCY STOP !!!')

        elif (message=='check'):
            self.checkStatus()

        elif (message=='ros_quit'):
            tmux_kill('-a')
            time.sleep(3)
            i=1
            while (i<=nwindows):
                tmux_Cc(i)  # C-c on all the windows
                time.sleep(1)
                i += 1
            i=1
            while (i<=nwindows):
                tmux_Ck(i)  # C-c on all the windows
                time.sleep(1)
                i += 1
            self.checkStatus()

        elif (message=='robot_start'):
            tmux_launch(1,'robot','robot')
            time.sleep(3)
            self.checkStatus()
        elif (message=='robot_kill'):
            tmux_kill('orazio')
            tmux_kill('state_pub_robot')
            time.sleep(3)
            self.checkStatus()

        elif (message=='simrobot_start'):
            tmux_launch(1,'stage','simrobot')
            time.sleep(3)
            self.checkStatus()
        elif (message=='simrobot_kill'):
            tmux_kill('stageros')
            time.sleep(3)
            self.checkStatus()

        elif (message=='usbcam_start'):
            tmux_launch(3,'camera','usbcam')
            time.sleep(3)
            self.checkStatus()
        elif (message=='usbcam_kill'):
            tmux_kill('usb_cam')
            tmux_kill('state_pub_usbcam')
            time.sleep(3)
            self.checkStatus()

        elif (message=='astra_start'):
            tmux_launch(3,'camera','astra')
            time.sleep(3)
            self.checkStatus()
        elif (message=='astra_kill'):
            tmux_kill('astra')
            tmux_kill('state_pub_astra')
            time.sleep(3)
            tmux_Cc(3)
            time.sleep(3)
            self.checkStatus()

        elif (message=='xtion_start'):
            tmux_launch(3,'camera','xtion')
            time.sleep(3)
            self.checkStatus()
        elif (message=='xtion_kill'):
            tmux_kill('xtion')
            tmux_kill('state_pub_xtion')
            time.sleep(3)
            tmux_Cc(3)
            time.sleep(3)
            self.checkStatus()

        elif (message=='hokuyo_start'):
            tmux_launch(2,'laser','hokuyo')
            time.sleep(3)
            self.checkStatus()
        elif (message=='hokuyo_kill'):
            tmux_kill('hokuyo')
            tmux_kill('state_pub_laser')
            time.sleep(3)
            self.checkStatus()

        elif (message=='rplidar_start'):
            tmux_launch(2,'laser','rplidar')
            time.sleep(3)
            self.checkStatus()
        elif (message=='rplidar_kill'):
            tmux_kill('rplidar')
            tmux_kill('state_pub_laser')
            time.sleep(3)
            self.checkStatus()

        elif (message=='astralaser_start'):
            tmux_launch(2,'laser','astra_laser')
            time.sleep(3)
            self.checkStatus()
        elif (message=='astralaser_kill'):
            tmux_kill('astra')
            tmux_kill('depth2laser')
            tmux_kill('state_pub_astra_laser')
            time.sleep(3)
            self.checkStatus()

        elif (message=='xtionlaser_start'):
            tmux_launch(2,'laser','xtion_laser')
            time.sleep(3)
            self.checkStatus()
        elif (message=='xtionlaser_kill'):
            tmux_kill('xtion')
            tmux_kill('depth2laser')
            tmux_kill('state_pub_xtion_laser')
            time.sleep(3)
            self.checkStatus()

        elif (message=='joystick_start'):
            tmux_launch(4,'teleop','teleop')
            time.sleep(3)
            self.checkStatus()
        elif (message=='joystick_kill'):
            tmux_kill('joystick')
            tmux_kill('joy')
            time.sleep(3)
            self.checkStatus()

        elif (message=='audio_start'):
            tmux_python(5,'audio','audio_server.py')
            time.sleep(3)
            self.checkStatus()
        elif (message=='audio_kill'):
            tmux_Cc(5)
            time.sleep(3)
            self.checkStatus()

        elif (message=='wsrobot_start'):
            tmux_python(6,'blockly','websocket_robot.py')
            time.sleep(3)
            self.checkStatus()
        elif (message=='wsrobot_kill'):
            tmux_Cc(6)
            time.sleep(3)
            tmux_Ck(6)
            time.sleep(3)
            self.checkStatus()


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

    tmux_init()

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
        print("-- Keyboard interrupt --")

    if (not websocket_server is None):
        websocket_server.close()
    print("Bringup Websocket server quit.")
    run = False    
    print("Waiting for main loop to quit...")


