# -*- coding: utf-8 -*-

import os, sys
import yaml


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

        if config['simulator']['stage']:
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


if __name__=='__main__':

    yamlfile = os.getenv('HOME')+"/system_config.yaml"

    config = readconfig(yamlfile)
    print("Config: "+str(config))

    arch = machinearch()
    print("Arch: %s" %arch)

    writeout(config, arch)


