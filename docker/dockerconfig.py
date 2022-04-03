#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os, sys
try:
    import yaml
except Exception as e:
    print(e)
    print("Try install yaml with    sudo apt-get install python3-yaml")
    sys.exit(1)

def readconfig(yamlfile):
    info = {}
    with open(yamlfile, 'r') as f:
        info = yaml.safe_load(f)
    return info

# Intel: x86_64, Raspberry 4: aarch64, Raspberry 3: armv7l
def machinearch():
    nfile = "/tmp/macharch"
    os.system("uname -m > %s" %nfile)
    with open(nfile, 'r') as f:
        info = f.readline().strip()
    return info

def robottype(config):
    rtype = None
    if config['simulator']['stage']:
        rtype = "stage"
    elif config['robot']['motorboard']!=False:
        rtype = "marrtino"

    os.system("echo \"%s\" > /tmp/robottype" %rtype)

    return rtype

def cameraresolution(config):
    camres = None
    if 'camera_resolution' in config['robot']:
        camres = config['robot']['camera_resolution']
    os.system("echo '%s' > /tmp/cameraresolution" %camres)
    os.system("cat /tmp/cameraresolution")

    return camres


def addservice(f, service, version=None):
    print(" - "+service)
    with open("docker-compose.%s" %service, 'r') as r:
        for l in r:
            if (l.strip()[0:6]=='image:' and version is not None):
                f.write(l[0:-1]+"%s\n" %version)
            else:
                f.write(l)


def writeout(config, arch):
    nfile = "docker-compose.yml"
    print("\nservices:")
    with open(nfile, 'w') as f:
        f.write("version: \"3.9\"\n\n")
        f.write("services:\n\n")

        addservice(f,'base')

        if config['system']['nginx']:
            addservice(f,'nginx')

        #  stage: [off|on|x11|vnc]
        cstage = config['simulator']['stage']
        if cstage == "vnc":
            addservice(f,'stage-vnc')
        elif cstage == True or cstage == "on" or cstage == "x11":
            addservice(f,'stage')
            

        # robot
        # motorboard: marrtino2019|pka03|ln298|arduino
        orazioversion = None
        if config['robot']['motorboard']!=False:
          # default
          if arch=='x86_64':
            orazioversion=""
          else:
            orazioversion=":arm64"
        if config['robot']['motorboard']=='arduino':
          if arch=='x86_64':
            orazioversion=":2018"
          else:
            orazioversion=":2018-arm64"
        if orazioversion != None:
            addservice(f,'orazio',orazioversion)

        if config['robot']['4wd']=='':
            pass

        if config['robot']['joystick']:
            addservice(f,'teleop')

        if config['robot']['camera']=='':
            pass

        if config['robot']['4wd']=='':
            pass

        if config['robot']['laser']!=False or config['functions']['navigation']:
            addservice(f,'navigation')

        if config['functions']['vision']:
            addservice(f,'vision')

        if config['functions']['speech']:
            addservice(f,'speech')

        if config['functions']['social']:
            os.system('touch /tmp/marrtinosocialon') 
            # used by start_docker.bash / system_update.bash
        else:
            os.system('rm -f /tmp/marrtinosocialon')



if __name__=='__main__':

    yamlfile = os.getenv('MARRTINO_APPS_HOME')+"/system_config.yaml"
    if not os.path.isfile(yamlfile):
        yamlfile = os.getenv('HOME')+"/system_config.yaml"
    if not os.path.isfile(yamlfile):
        print("File system_config.yaml not found!!!")
        sys.exit(1)    

    config = readconfig(yamlfile)
    print("Config: "+str(config))

    arch = machinearch()
    print("Arch: %s" %arch)

    rtype = robottype(config)
    print("Robot: %s" %rtype)

    camres = cameraresolution(config)
    print("Camera resolution: %s" %(camres))

    writeout(config, arch)


