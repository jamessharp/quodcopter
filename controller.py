import serial
import time
import json
import threading
import Queue
import cmd
import sys

commsQueue = Queue.Queue()

class ComThread(threading.Thread):
    def __init__(self, threadID, name, device, baud):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.device = device
        self.baud = baud
        self.ser = serial.Serial(device, baud)
        self.lastHeartbeat = int(round(time.time() * 1000))

    def doHeartbeat(self):
        now = int(round(time.time() * 1000))
        if (now - self.lastHeartbeat) > 500:
            self.lastHeartbeat = now
            command = json.dumps({'cmd': 'heartbeat'})
            self.ser.write(command)

    def run(self):
        while True:
            self.doHeartbeat()
            try:
                item = commsQueue.get(False)
                command = json.dumps(item)
                self.ser.write(command)
                commsQueue.task_done()
            except Queue.Empty:
                pass
            time.sleep(0.01)

class QuadCmd(cmd.Cmd):

    def __init__(self):
        cmd.Cmd.__init__(self)
        self.prompt = '> '

    def do_f(self, forceStr):
        force = int(forceStr)
        command = {'cmd': 'force', 'val': force}
        commsQueue.put(command)

    def do_q(self, s):
        sys.exit(1)

    def do_h(self, s):
        command = {'cmd': 'hover'}
        commsQueue.put(command)

if __name__ == "__main__":
    comThread = ComThread(1, 'ComThread', '/dev/tty.usbmodem1411', 9600)
    comThread.daemon = True

    # Start the com thread
    comThread.start()

    cli = QuadCmd()
    cli.cmdloop()

    while threading.active_count() > 0:
        time.sleep(0.1)


