# ActionProxy base class
#

from threading import Thread

class ActionProxy:

    def __init__(self):
        self.do_run = False
        self.action_thread = None

    def __del__(self):
        # just in case
        self.end()

    def start(self, params=None):
        if self.action_thread != None:
            self.end()
        self.do_run = True
        self.action_thread = Thread(target=self.monitor_thread, args=(params,))
        self.action_thread.start()

    def interrupt(self):
        self.end()

    def end(self):
        self.do_run = False
        if self.action_thread != None:
            self.action_thread.join()
        self.action_thread = None

    def isRunning(self):
        self.do_run = self.action_thread != None and self.action_thread.is_alive()
        return self.do_run

    def monitor_thread(self, params):
        pass




