# wait action

import time, sys

from actionproxy import ActionProxy

ACTION_NAME = 'wait'

class WaitActionProxy(ActionProxy):

    def __init__(self, actionname):
        ActionProxy.__init__(self, actionname)

    def monitor_thread(self, params):
        self.cnt = 0
        timeout = int(params)
        while self.do_run and self.cnt<timeout:
            time.sleep(1)
            self.cnt += 1
            print("Action: %s - status: %d" %(self.actionname,self.cnt))


if __name__ == "__main__":

    params = None
    if (len(sys.argv)>1):
        params = sys.argv[1]

    wait = WaitActionProxy(ACTION_NAME)
    
    if params is not None:
        wait.execute(params)  # blocking, CTRL-C to interrupt
    else:
        wait.run_server()     # blocking, CTRL-C to interrupt

