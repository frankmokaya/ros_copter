#!/usr/bin/env python

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

from geometry_msgs.msg import Twist
from fcu_common.msg import MR_Controller_Commands

import sys, select, termios, tty

msg = """
Control Your Turtlebot!
---------------------------
Translating:
   u    i    o
   j    k    l
   m    ,    .

 Turning/Climbing
        w
   a    s    d
        x


q/e : increase/decrease max speeds by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
            # x, y, yaw, z
        'i':(1, 0, 0, 0),
        'o':(1, 1, 0, 0),
        'j':(0, -1, 0, 0),
        'l':(0, 1, 0, 0),
        'u':(1, -1, 0, 0),
        ',':(-1, 0, 0, 0),
        '.':(-1, 1, 0, 0),
        'm':(-1, -1, 0, 0),

        'w':(0, 0, 0, 1),
        'x':(0, 0, 0, -1),
        'a':(0, 0, -1, 0),
        'd':(0, 0, 1, 0)
         }

speedBindings={
        'q':(1.1,1.1,1.1,1.1),
        'e':(.9,.9,.9,.9)
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .2
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('key_teleop')
    pub = rospy.Publisher('command', MR_Controller_Commands, queue_size=5)

    x = 0
    y = 0
    psi = 0
    z = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print msg
        print vels(speed,turn)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                psi = moveBindings[key][2]
                z = z + 0.1*speed*moveBindings[key][3]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                count = 0

                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                x = 0
                y = 0
                psi = 0
                z = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    y = 0
                    psi = 0
                if (key == '\x03'):
                    break

            command = MR_Controller_Commands()
            command.velocity.x = x*speed
            command.velocity.y = y*speed
            command.omega.z = psi*speed
            command.position.z = z
            command.velocity_valid = 1
            pub.publish(command)


            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except:
        print('error')

    finally:
        command = MR_Controller_Commands()
        command.velocity.x = 0
        command.velocity.y = 0
        command.omega.z = 0
        command.position.z = z
        command.velocity_valid = 1

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
