import sys
import socket
import time

ip = '127.0.0.1'
port = 9001

if (len(sys.argv)>1):
    ip = sys.argv[1]
if (len(sys.argv)>2):
    port = int(sys.argv[2])

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

sock.connect((ip,port))

sock.send('start\n\r')
time.sleep(1)
sock.send('TTS ciao, come stai?\n\r')

sock.close()

