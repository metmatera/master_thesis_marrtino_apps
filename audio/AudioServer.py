# sudo -H pip install pyglet

# Only PCM 16 bit wav 44100 Hz - Use audacity to convert audio files.

# WAV generation

# Synth
# sox -n --no-show-progress -G --channels 1 -r 44100 -b 16 -t wav bip.wav synth 0.25 sine 800 
# sox -n --no-show-progress -G --channels 1 -r 44100 -b 16 -t wav bop.wav synth 0.25 sine 400 

# Voices
# pico2wave -l "it-IT" -w start.wav "Bene! Si Parte!"

# To run this from a remote shell use:  'ssh -X ubuntu@<IP>'


import pyglet
import threading
import time
import socket
import sys


class AudioServer(threading.Thread):

	def __init__(self, port):
		threading.Thread.__init__(self)

		# Initialize pyglet player
		self.player = pyglet.media.Player()
		self.player.play()

		# Create a TCP/IP socket
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.sock.settimeout(3)
		# Bind the socket to the port
		server_address = ('localhost', port)
		self.sock.bind(server_address)
		self.sock.listen(1)
		print "AudioServer running on port ", port, " ..."
		
		self.dorun = True
		self.connection = None

		# Dictionary of sounds
		self.Sounds = {}
		#self.Sounds["bip"] = pyglet.resource.media("bip.wav", streaming=False)  # load in memory
		#self.Sounds["bip"].play()

	def stop(self):
		self.dorun = False

	def connect(self):
		connected = False
		while (self.dorun and not connected):
			try:
				# print 'Waiting for a connection ...'
				# Wait for a connection
				self.connection, client_address = self.sock.accept()
				self.connection.settimeout(3)
				connected = True
				print 'Connection from ', client_address
			except:
				pass #print "Listen again ..."	


	def run(self):
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
						print 'Received "%s"' % data
						self.play(data)
						#print 'sending data back to the client'
						#self.connection.sendall("OK")
					elif (data == None or data==""):
						break     
			finally:
				print 'Connection closed.'
				# Clean up the connection
				if (self.connection != None):
					self.connection.close()
					self.connection = None


	def play(self, name):
		print "Playing ",name
		if (not name in self.Sounds):
			try:
				self.Sounds[name] = pyglet.resource.media(name+".wav", streaming=False)  # False: load in memory
			except:
				print "File %s.wav not found." %(name)

		if (name in self.Sounds):
			self.Sounds[name].play()



if __name__ == "__main__":
	port = 9001
	if (len(sys.argv)>1):
		port = int(sys.argv[1]);

	server = AudioServer(port)
	server.start()

	try:
		pyglet.app.run()
	except:
		print "Exit"
		server.stop()
		sys.exit(0)





