#!/usr/bin/env python3

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from std_msgs.msg import Char
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios



class StateMachine(object):
    def __init__(self):
        self.key = None
        self.current_state = 0
        self.state_publisher = rospy.Publisher("/turdel_state", Char, queue_size=2)
        self.msg = """
            Control Your TurtleBot3!
            ---------------------------
            Switch Control Mode:

            w/W == WallFollowing
            o/O == ObstacleAvoidance
            l/L == LineFollowing

            space key, s : force stop

            CTRL-C to quit
            """
        self.e = """
            Communications Failed
            """

    def getKey(self):
        if os.name == 'nt':
            timeout = 0.1
            startTime = time.time()
            while(1):
                if msvcrt.kbhit():
                    if sys.version_info[0] >= 3:
                        return msvcrt.getch().decode()
                    else:
                        return msvcrt.getch()
                elif time.time() - startTime > timeout:
                    return ''

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            self.key = sys.stdin.read(1)
        else:
            self.key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return self.key

    def reportState(self):
        rospy.loginfo("the current state is: {0}".format(self.current_state))

    def main(self):
        print(self.msg)
        while not rospy.is_shutdown():
            state_machine.key = state_machine.getKey()
            if state_machine.key == 'w' or state_machine.key == 'W':
                state_machine.current_state = 1
                state_machine.reportState()
                state_machine.state_publisher.publish(state_machine.current_state)
            elif state_machine.key == 'o' or state_machine.key == 'O' :
                state_machine.current_state = 2
                state_machine.reportState()
                state_machine.state_publisher.publish(state_machine.current_state)
            elif state_machine.key == 'l' or state_machine.key == 'L' :
                state_machine.current_state = 3
                state_machine.reportState()
                state_machine.state_publisher.publish(state_machine.current_state)
            else:
                if (key == '\x03'):
                    break


if __name__=="__main__":
    print("in main main")
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turdel_state_node')
    state_machine = StateMachine()
    state_machine.reportState()

    while not rospy.is_shutdown():
        try:
            state_machine.main()

                # if status == 20 :
                #     print(msg)
                #     status = 0

        except:
            print(state_machine.e)

        finally:
            state_machine.current_state = 0

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)