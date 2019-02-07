import os
import time

sessionname = 'navigation'

def send_command(cmd, name):
    print cmd
    use_tmux = os.system('tmux -V')==0
    
    if use_tmux:        
        r = os.system('tmux select-window -t %s:0' %sessionname)
        if r!=0:
            os.system('tmux -2 new-session -d -s %s' %sessionname)
        r = os.system('tmux select-window -t %s' %name)
        if (r!=0): # if not existing
            os.system('tmux new-window -n %s' %name)
        os.system('tmux send-keys "%s" C-m' %cmd)
    else:
        os.system('xterm -hold -e "%s" &' %cmd)

def main():
    send_command('cd $MARRTINO_APPS_HOME/stage','stage')
    send_command('roslaunch simrobot.launch','stage')
    time.sleep(3)
    send_command('cd $MARRTINO_APPS_HOME/navigation','loc')
    send_command('roslaunch srrg_localizer.launch map_name:=montreal initial_pose_x:=2.5 initial_pose_y:=11.0','loc')
    time.sleep(3)
    send_command('cd $MARRTINO_APPS_HOME/navigation','nav')
    send_command('roslaunch move_base.launch','nav')
    time.sleep(1)



if __name__ == '__main__':
    main()

