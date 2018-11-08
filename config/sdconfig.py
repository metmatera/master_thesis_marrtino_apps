import time
import os

import sys
sys.path.append('../bringup')

from tmuxsend import TmuxSend

#        Device Boot      Start         End      Blocks   Id  System
#/dev/mmcblk0p1   *        2048      133119       65536    c  W95 FAT32 (LBA)
#/dev/mmcblk0p2          133120    20613119    10240000   83  Linux
#/dev/mmcblk0p3        20613120    63272959    21329920   83  Linux


def umount(tmux):
    time.sleep(5)
    tmux.cmd(1,'df -h')
    tmux.cmd(1,'umount /dev/mmcblk0p1',1)
    tmux.cmd(1,'umount /dev/mmcblk0p2',1)
    tmux.cmd(1,'umount /dev/mmcblk0p3',1)
    tmux.cmd(1,'df -h')
    time.sleep(3)


def mount(tmux):
    tmux.cmd(1,'mkdir /media/$SUDO_USER/PI_ROOT')
    tmux.cmd(1,'mount /dev/mmcblk0p2 /media/$SUDO_USER/PI_ROOT',1)
    tmux.cmd(1,'df -h')
    time.sleep(3)


def sudobash(tmux):
    tmux.cmd(1,'sudo bash')
    time.sleep(1)
    tmux.cmd(1,'marrtino')
    time.sleep(1)

def format(tmux):

    #sudobash(tmux)
    umount(tmux)

    tmux.cmd(1,'fdisk /dev/mmcblk0',2)
    tmux.cmd(1,'p\no\nw\n')
    time.sleep(1)
    tmux.cmd(1,'fdisk /dev/mmcblk0',2)
    tmux.cmd(1,'n\np\n1\n2048\n133119\nt\nc\np\n')
    time.sleep(1)
    tmux.cmd(1,'n\np\n2\n133120\n20613119\np\n')
    time.sleep(1)
    tmux.cmd(1,'n\np\n3\n\n\np\nw\n',3)

    umount(tmux)

    tmux.cmd(1,'mkfs.fat -n PI_BOOT /dev/mmcblk0p1',3)
    tmux.cmd(1,'mkfs.ext4 -L PI_ROOT /dev/mmcblk0p2',15)
    tmux.cmd(1,'mkfs.ext4 -L data /dev/mmcblk0p3',15)
    time.sleep(3)

    tmux.cmd(1,'fdisk /dev/mmcblk0',2)
    tmux.cmd(1,'a\n1\np\nw\n',3)

    umount(tmux)

def copyimg(tmux):
    sudobash(tmux)
    # ./writeimg.bash ....


def checkimg(tmux):
    #sudobash(tmux)
    mount(tmux)
    mdir = '/media/$SUDO_USER/PI_ROOT/'
    tmux.cmd(1,'cd %s/etc/NetworkManager/system-connections/' %mdir)
    tmux.cmd(1,'ls')
    tmux.cmd(1,'cat %s/etc/hostname' %mdir)
    tmux.cmd(1,'cat %s/etc/hosts' %mdir)
    time.sleep(3)
    tmux.cmd(1,'cat %s/home/ubuntu/.bashrc | grep ROS_IP' %mdir)
    time.sleep(1)
    tmux.cmd(1,'cd $HOME')
    umount(tmux)


# Main program

if __name__ == "__main__":
    tmux = TmuxSend('formatsd',['format'])
    #format(tmux)
    checkimg(tmux)

