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
from std_msgs.msg import Int16
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios


msg = """
Control Your TurtleBot3!
---------------------------
Switch Control Mode:

w/W == WallFollowing
o/O == ObstacleAvoidance
l/L == LineFollowing

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
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
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
 
    rospy.init_node('turtle_state_node')
    pub = rospy.Publisher('turtle_state', Int16, queue_size=10)
    state = 0


    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'w' or key == 'W' or key == 'o' or key == 'O':
                print("switching to wall follwoing/obstacle avoidance")
                state = 1
            elif key == 'l' or key== 'L' :
                print("switching to line following")
                state = 2
            elif key == 's' or key == 'S' or key == ' ':
                state = 0
                print("stopping MazeRunner")
            else:
                if (key == '\x03'):
                    break

            pub_state = Int16()
            pub_state.data = state


            pub.publish(pub_state)

    except:
        print(e)

    finally:
        pub_state = Int16()
        pub_state.data = 0
        pub.publish(pub_state)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)