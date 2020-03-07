import os

URLbase = 'http://www.diag.uniroma1.it/~iocchi/marrtino/'
updatesfile = 'updates.txt'


def getversion(upstr):
    r = ''
    if ('marrtino_update' in upstr):
        vstr = upstr.split('_')
        l = len(vstr[2])
        r = vstr[2][1:l-5]
    return r


# cd ~/install'
os.chdir(os.getenv('HOME')+'/install')
os.system('mkdir -p .log')

cmd = 'wget -N '+URLbase+updatesfile
os.system(cmd)

ever = os.getenv('MARRTINO_VERSION')

fmv = open(os.getenv('HOME')+'/.marrtino_version')
fver = fmv.read().strip()
fmv.close()

if (ever>fver):
    cver = ever
else:
    cver = fver


print("Current version: %s" %cver)

upfile = open(updatesfile,"r")
lines = upfile.readlines()
vmax = ''
for l in lines:
    v = getversion(l.strip())
    if (v>cver):    
        print('New version available: %s' %v)
        if (vmax==''):
            vmax=v
upfile.close()

logfile = os.getenv('HOME')+"/install/.log/install_`date +%Y%m%d_%H%M%S`.log"

if (vmax!=''):
    val = raw_input('Install version %s? [y/n] ' %(vmax))

    if (str(val)=='y' or str(val)=='Y'):
        print("Installing marrtino v%s ..." %(vmax))
        upcmd = 'marrtino_update_v%s.bash' %(vmax)
        cmd = 'wget -N '+URLbase+upcmd
        print(cmd)
        os.system(cmd)
        cmd = 'chmod a+x '+upcmd
        print(cmd)
        os.system(cmd)
        cmd = './'+upcmd
        print(cmd)
        print('Installation in progres... please wait...')
        print('See installation output on another terminal with command:')
        print('tail -f '+logfile)
        os.system(cmd + " &> " + logfile)
        print('Installation completed succesfully.')

else:
    print('Marrtino is currently updated to the last version.')



