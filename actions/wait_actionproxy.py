# wait action

import time, sys

from actionproxy import ActionProxy

class WaitActionProxy(ActionProxy):

    def __init__(self):
        ActionProxy.__init__(self)
        self.cnt = 0

    def monitor_thread(self, params):
        timeout = int(params)
        while self.do_run and self.cnt<timeout:
            time.sleep(1)
            self.cnt += 1
            print("Wait action: %d" %self.cnt)


if __name__ == "__main__":

    params = "10"
    if (len(sys.argv)>1):
        params = sys.argv[1]

    wait = WaitActionProxy()
    wait.start(params)

    while (wait.isRunning()):
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            wait.interrupt()
    
    wait.end()


