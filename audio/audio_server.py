
# Only PCM 16 bit wav 44100 Hz - Use audacity or sox to convert audio files.

# WAV generation

# Synth
# sox -n --no-show-progress -G --channels 1 -r 44100 -b 16 -t wav bip.wav synth 0.25 sine 800 
# sox -n --no-show-progress -G --channels 1 -r 44100 -b 16 -t wav bop.wav synth 0.25 sine 400 

# Voices
# pico2wave -l "it-IT" -w start.wav "Bene! Si Parte!"
# Then convert wav files to to 44100 Hz

# Note: some initial sound may not be played.


import threading
import time
import socket
import sys
import os
import re
import wave
import argparse

try:
    import pyaudio
except:
	print('pyaudio required. Install with:   sudo apt install python-pyaudio')
	sys.exit(0)

try:
    import sox
except:
	print('sox required. Install with:   sudo -H pip install sox')
	sys.exit(0)


from asr_server import ASRServer



SOUNDS_DIR = "sounds/"  # dir with sounds
soundfile = None        # sound file


tts_server = None
asr_server = None


def TTS_callback(in_data, frame_count, time_info, status):
    global soundfile
    if (soundfile==None):
        return (None, True)
    else:
        data = soundfile.readframes(frame_count)
        return (data, pyaudio.paContinue)


class TTSServer(threading.Thread):

    def __init__(self, port, output_device):
        threading.Thread.__init__(self)

        # Initialize audio player
        self.pa = pyaudio.PyAudio()  
        self.streaming = False
        self.output_device = output_device

        print("Audio devices available")
        for dd in range(self.pa.get_device_count()):
            for di in [self.pa.get_device_info_by_index(dd)]:
                print "   ",dd,di['name']
                if ((di['name']=='default') and (self.output_device < 0)):
                    self.output_device = dd # choose default device
        print("Audio device used: %d" %self.output_device)

        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(3)
        # Bind the socket to the port
        server_address = ('', port)
        self.sock.bind(server_address)
        self.sock.listen(1)
        print "TTS Server running on port ", port, " ..."
        
        self.dorun = True
        self.connection = None

        # Dictionary of sounds
        self.Sounds = {}
        self.Sounds['bip'] = wave.open(SOUNDS_DIR+'bip.wav', 'rb') 
        self.idcache = 0
    
    def stop(self):
        self.dorun = False

    def connect(self):
        connected = False
        while (self.dorun and not connected):
            try:
                # print 'Waiting for a connection ...'
                # Wait for a connection
                self.connection, client_address = self.sock.accept()
                self.connection.settimeout(3) # timeout when listening (exit with CTRL+C)
                connected = True
                print 'TTS Server Connection from ', client_address
            except:
                pass #print "Listen again ..."    


    def run(self):
        global asr_server
        for i in range(0,1):
            print 'bip'
            self.play('bip')
            time.sleep(1)
        while (self.dorun):
            self.connect()
            try:
                # Receive the data in small chunks and retransmit it
                while (self.dorun):
                    try:
                        data = self.connection.recv(80)
                        data = data.strip()
                    except socket.timeout:
                        data = "***"
                    except:
                        data = None
                    
                    if (data!=None and data !="" and data!="***"):
                        print 'TTS Received "%s"' % data
                        if (data.startswith('TTS')):
                            lang = 'en-US' # default language
                            strsay = data[4:]
                            if (data[3]=='['):
                                vd = re.split('\[|\]',data)
                                lang = vd[1]
                                strsay = vd[2]
                            self.say(strsay,lang)
                        elif (data=="ASR"):
                            print('asr request')
                            bh = asr_server.get_asr()
                            self.connection.send(bh+'\n\r')
                            print('asr sent %s' %bh)

                        else:
                            self.play(data)
                        #print 'sending data back to the client'
                        #self.connection.sendall("OK")
                    elif (data == None or data==""):
                        break     
            finally:
                print 'TTS Server Connection closed.'
                # Clean up the connection
                if (self.connection != None):
                    self.connection.close()
                    self.connection = None



    def say(self, data, lang):
        print 'Say ',data
        cachefile = 'cache'+str(self.idcache)
        self.idcache = (self.idcache+1)%10
        tmpfile = "/tmp/cache.wav"
        ofile = "%s%s.wav" %(SOUNDS_DIR, cachefile)
        cmd = 'rm %s %s' %(tmpfile, ofile)
        os.system(cmd)
        time.sleep(0.1)
        cmd = 'pico2wave -l "%s" -w %s " , %s"' %(lang,tmpfile, data)
        print cmd
        os.system(cmd)
        time.sleep(0.1)

        # convert samplerate
        tfm = sox.Transformer()
        tfm.rate(samplerate=44100)
        tfm.build(tmpfile, ofile)
        time.sleep(0.1)

        self.play(cachefile)


    def play(self, name):
        print 'Play ',name
        i = 0
        while ((not name in self.Sounds) and (i<3)):
            try:
                self.Sounds[name] = wave.open(SOUNDS_DIR+name+".wav", 'rb')
            except:
                print "File %s%s.wav not found." %(SOUNDS_DIR,name)
                time.sleep(0.5)
            i += 1
        if (name in self.Sounds):
            self.playwav2(self.Sounds[name])
            #time.sleep(1)
        if (self.connection != None):
            self.connection.send('OK')

    def playwav(self, soundfile):
        chunk = 2048
        data = soundfile.readframes(chunk)
        while (len(data)>0):
            self.stream.write(data)  
            data = soundfile.readframes(chunk)  

    def playwav2(self, sfile):
        global soundfile
        self.streaming = True
        self.stream = self.pa.open(format = 8, #self.pa.get_format_from_width(f.getsampwidth()),  
                channels = 1, #f.getnchannels(),  
                rate = 44100, #f.getframerate(),  
                output = True,
                stream_callback = TTS_callback,
                output_device_index = self.output_device)
        soundfile = sfile
        soundfile.setpos(0)
        self.stream.start_stream()
        while self.stream.is_active():
            time.sleep(1.0)
        self.stream.stop_stream()  
        self.stream.close()  
        self.streaming = False




if __name__ == "__main__":


    parser = argparse.ArgumentParser(description='audio_server')
    parser.add_argument('-ttsport', type=int, help='TTS server port [default: 9001]', default=9001)
    parser.add_argument('-asrport', type=int, help='ASR server port [default: 9002]', default=9002)
    parser.add_argument('-device', type=int, help='audio device id [default: -1 = default]', default=-1)

    args = parser.parse_args()

    tts_server = TTSServer(args.ttsport,args.device)
    tts_server.start()

    asr_server = ASRServer(args.asrport)
    asr_server.start()

    run = True
    while (run):
        try:
            time.sleep(3)
            if (not tts_server.streaming):
                cmd = 'play -n --no-show-progress -r 44100 -c1 synth 0.1 sine 50 vol 0.01' # keep sound alive
                os.system(cmd)
        except KeyboardInterrupt:
            print "Exit"
            run = False

    tts_server.stop()
    asr_server.stop()
    sys.exit(0)





