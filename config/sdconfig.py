import time
import os
import argparse
import sys
sys.path.append('../bringup')

devicename = '/dev/mmcblk0'

from tmuxsend import TmuxSend

#        Device Boot      Start         End      Blocks   Id  System
#/dev/mmcblk0p1   *        2048      133119       65536    c  W95 FAT32 (LBA)
#/dev/mmcblk0p2          133120    20613119    10240000   83  Linux
#/dev/mmcblk0p3        20613120    63272959    21329920   83  Linux


#Device         Boot    Start      End  Sectors  Size Id Type
#/dev/mmcblk0p1 *        2048   133119   131072   64M  e W95 FAT16 (LBA)
#/dev/mmcblk0p2        133120 20613119 20480000  9,8G 83 Linux
#/dev/mmcblk0p3      20613120 61030399 40417280 19,3G 83 Linux


def umount(tmux):
    #tmux.cmd(1,'df -h')
    tmux.cmd(1,'umount %sp1' %devicename,blocking=True)
    tmux.cmd(1,'umount %sp2' %devicename,blocking=True)
    tmux.cmd(1,'umount %sp3' %devicename,blocking=True)
    #tmux.cmd(1,'df -h')
    time.sleep(3)


def mount(tmux):
    tmux.cmd(1,'mkdir /media/$SUDO_USER/PI_ROOT',blocking=True)
    tmux.cmd(1,'mount %sp2 /media/$SUDO_USER/PI_ROOT' %devicename,blocking=True)
    #tmux.cmd(1,'df -h')
    time.sleep(3)


def sudobash(tmux):
    tmux.cmd(1,'sudo bash')
    time.sleep(1)
    tmux.cmd(1,'marrtino')
    time.sleep(1)

def format(tmux):
    print('Formatting ...')

    wid = 1
    umount(tmux)
    tmux.cmd(wid,'fdisk %s' %devicename,2)
    tmux.cmd(wid,'p\no\nw\n')
    time.sleep(1)
    tmux.cmd(wid,'fdisk %s' %devicename,2)
    tmux.cmd(wid,'n\np\n1\n2048\n133119\nt\ne\np\n')
    time.sleep(1)
    tmux.cmd(wid,'n\np\n2\n133120\n20613119\np\n')
    time.sleep(1)
    tmux.cmd(wid,'n\np\n3\n\n\np\nw\n',3)

    umount(tmux)

    tmux.cmd(wid,'mkfs.fat -n PI_BOOT %sp1' %devicename,blocking=True)
    tmux.cmd(wid,'mkfs.ext4 -F -L PI_ROOT %sp2' %devicename,blocking=True)
    tmux.cmd(wid,'mkfs.ext4 -F -L data %sp3' %devicename,blocking=True)
    time.sleep(3)

    tmux.cmd(wid,'fdisk %s' %devicename,2)
    tmux.cmd(wid,'a\n1\np\nw\n',3)

    umount(tmux)

def write(tmux, imagefile):
    print('Writing ...')
    #umount(tmux)
    wid=2

    #tmux.cmd(wid,'./writeimg.bash %s' %imagefile, blocking=True)

    cmd = 'dd if=%s_boot.img bs=512 of=%sp1' %(imagefile,devicename)
    tmux.cmd(wid, cmd, blocking=True)
    cmd = 'dd if=%s_root.img | pv -s 11G | dd bs=512 of=%sp2' %(imagefile,devicename)
    tmux.cmd(wid, cmd, blocking=True)
    tmux.cmd(wid, 'sudo sync', blocking=True)

    umount(tmux)
    

def check(tmux):
    print('Checking ...')
    wid=3
    mount(tmux)
    mdir = '/media/$SUDO_USER/PI_ROOT/'
    tmux.cmd(wid,'cd %s/etc/NetworkManager/system-connections/' %mdir)
    tmux.cmd(wid,'ls')
    tmux.cmd(wid,'cat MARRtinoAP')
    tmux.cmd(wid,'cat %s/etc/hostname' %mdir)
    tmux.cmd(wid,'cat %s/etc/hosts' %mdir)
    time.sleep(3)
    tmux.cmd(wid,'cat %s/home/ubuntu/.bashrc | grep ROS_IP' %mdir)
    time.sleep(1)
    tmux.cmd(wid,'cd $HOME')
    umount(tmux)


def test(tmux):
    wid=3
    tmux.cmd(wid,'echo sleep 10 non blocking...')
    tmux.cmd(wid,'date')
    tmux.cmd(wid,'sleep 10',1)
    tmux.cmd(wid,'date')
    tmux.cmd(wid,'echo done')

    tmux.cmd(wid,'echo sleep 10 blocking...')
    tmux.cmd(wid,'date')
    tmux.cmd(wid,'sleep 10',blocking=True)
    tmux.cmd(wid,'date')
    tmux.cmd(wid,'echo done')



# Main program

if __name__ == "__main__":

    if os.geteuid() != 0:
        exit("You need to have root privileges to run this script.")

    parser = argparse.ArgumentParser(description='MARRtino SD config')
    parser.add_argument('imagefile', type=str, help='image file prefix (e.g., raspi3b_marrtino_v2.0)')
    args = parser.parse_args()
    tmux = TmuxSend('sdconfig',['format','write','check'])

    print('Format SD card with image file %s' %args.imagefile)
    print('ALL DATA FROM SD CARD WILL BE ERASED!!!')
    val = raw_input('Please confirm [yes/no] ')
    if val=='yes':
        print("Running  'sudo tmux a -t sdconfig'  in a new terminal to check progresses.")
        os.system('xterm -e "sudo tmux a -t sdconfig" &')
        format(tmux)
        write(tmux,args.imagefile)
        check(tmux)
        print('Done')

