#!/usr/bin/env python

from __future__ import print_function

import threading

import rospy

from std_msgs.msg import String

import sys, select, termios, tty

gui = """
bimatrix controls:
u/j : increase/decrease amplitude
i/k : increase/decrease pulse width
o/l : increase/decrease repetition rate
y/h : increase/decrease burst duration
r/f : increase/decrease number of pairs

t   : toggle pulse generator
m   : change mode
CTRL-C to quit
"""

keyBindings = {
        'u':"a1",
        'j':"a0",
        'i':"w1",
        'k':"w0",
        'o':"r1",
        'l':"r0",
        'y':"d1",
        'h':"d0",
        'r':"p1",
        'f':"p0",
        't':"t",
        'm':"m"
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/keyboard', String, queue_size = 1)
        self.msg = "null"
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, msg):
        self.condition.acquire()
        self.msg = msg
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update("exit")
        self.join()

    def run(self):
        string_msg = String()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            string_msg.data = self.msg

            self.condition.release()

            # Publish.
            self.publisher.publish(string_msg)

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_publisher')

    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    msg = "0"

    try:
        pub_thread.wait_for_subscribers()

        print(gui)
        while(1):
            key = getKey(key_timeout)
            if key in keyBindings.keys():
                msg = keyBindings[key]
            
            else:
                if (key == '\x03'):
                    break

            pub_thread.update(msg)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        
