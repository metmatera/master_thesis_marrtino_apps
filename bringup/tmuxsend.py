import os
import time

class TmuxSend:

    # listwindows = [ 'windowname', ... ]
    def __init__(self, sessionname, listwindows):
        self.nwindows = len(listwindows)
        self.sessionname = sessionname
        os.system('tmux -2 new-session -d -s %s' %self.sessionname) # tmux session
        os.system('tmux select-window -t %s:0' %self.sessionname)
        os.system('tmux rename-window \'quit\'')                  # window 0 - quit
        for i in range(0,self.nwindows):
            os.system('tmux new-window -t %s:%d -n \'%s\'' %(self.sessionname, i+1, listwindows[i]))    

    def roslaunch(self, wid, mdir, mlaunch, mparams=''):
        os.system('tmux select-window -t %s:%d' %(self.sessionname,wid))
        os.system('tmux send-keys "cd $MARRTINO_APPS_HOME/%s" C-m' %(mdir))
        os.system('tmux send-keys "roslaunch %s.launch %s" C-m' %(mlaunch, mparams))

    def roscore(self, wid):
        os.system('tmux select-window -t %s:%d' %(self.sessionname,wid))
        os.system('tmux send-keys "roscore" C-m')

    def roskill(self, rosnode):
        wid = 0
        os.system('tmux select-window -t %s:%d' %(self.sessionname,wid))
        os.system('tmux send-keys "rosnode kill %s" C-m' %(rosnode))

    def python(self, wid, mdir, mpy, mparams=''):
        os.system('tmux select-window -t %s:%d' %(self.sessionname,wid))
        os.system('tmux send-keys "cd $MARRTINO_APPS_HOME/%s" C-m' %(mdir))
        os.system('tmux send-keys "python %s %s" C-m' %(mpy, mparams))

    def cmd(self, wid, cmd, sleeptime=0.1, blocking=False):
        os.system('tmux select-window -t %s:%d' %(self.sessionname,wid))
        if blocking:
            os.system('tmux send-keys "%s; sleep 1; tmux wait-for -S tmux-end" C-m' %(cmd))
            self.waitfor('tmux-end')            
        else:
            os.system('tmux send-keys "%s" C-m' %(cmd))
            time.sleep(sleeptime)


    def waitfor(self, wforlabel):
        #print('Waiting for tmux laber %s ...' %wforlabel)
        os.system('tmux wait-for %s' %(wforlabel))
        #print('  ... done')

    def killall(self, wid):
        self.Cc(wid)
        time.sleep(3)
        self.Ck(wid)

    def Cc(self, wid):
        os.system('tmux select-window -t %s:%d' %(self.sessionname,wid))
        os.system('tmux send-keys C-c')

    def Ck(self, wid):
        os.system('tmux select-window -t %s:%d' %(self.sessionname,wid))
        os.system('tmux send-keys C-\\')

    def quitall(self): # kill all processes in windows 1..n-1 (excluding 0 and n)
        self.roskill('-a')
        time.sleep(3)
        for i in range(0,self.nwindows-1):
            self.Cc(i+1)  # C-c on all the windows
            time.sleep(1)
        for i in range(0,self.nwindows-1):
            self.Ck(i+1)  # C-\ on all the windows
            time.sleep(1)


