#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

ROBOTINO_MAX_LIN_VEL = 0.2
ROBOTINO_MAX_ANG_VEL = 0.8

LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your Robotino!
---------------------------
Moving around:
   q     w    e
   a     s    d
         x

space key, x : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      if sys.version_info[0] >= 3:
        return msvcrt.getch().decode()
      else:
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -ROBOTINO_MAX_LIN_VEL, ROBOTINO_MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -ROBOTINO_MAX_ANG_VEL, ROBOTINO_MAX_ANG_VEL)
    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('robotino_control', anonymous=True)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    #turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel   = 0.0
    target_linear_vel_y = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_linear_vel_y  = 0.0
    control_angular_vel = 0.0


    try:
        print(msg)
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 's' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'a' : 
                target_linear_vel_y = checkLinearLimitVelocity(target_linear_vel_y + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel_y,target_angular_vel))
            elif key == 'd' : 
                target_linear_vel_y = checkLinearLimitVelocity(target_linear_vel_y - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel_y,target_angular_vel))
            elif key == 'q' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'e' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == ' ' or key == 'x' :
                target_linear_vel   = 0.0
                target_linear_vel_y = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                control_linear_vel_y  = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print(msg)
                status = 0

            twist = Twist()

            control_linear_vel_x = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            control_linear_vel_y = makeSimpleProfile(control_linear_vel_y, target_linear_vel_y, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel_x; twist.linear.y = control_linear_vel_y; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            cmd_vel_pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)