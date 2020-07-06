#!python
from __future__ import print_function
import RPi.GPIO as GPIO
import rospy , sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Teleop receiver node

class Robot:
    def __init__(self, rosmode = False):
        print("Starting teleop reveicer node..")
        # Pinout for motors
        self.a1 = 19 ; self.a2 = 26; self.a_pwm_pin = 13;
        self.b1 = 16 ; self.b2 = 20; self.b_pwm_pin = 12;
        self.wheel_omega = 100; self.wheel_omega_rot = 75;
                
        # Set modes
        GPIO.setmode(GPIO.BCM)
        for p in [self.a1,self.a2,self.b1,self.b2, self.a_pwm_pin, self.b_pwm_pin] : GPIO.setup(p, GPIO.OUT)
        self.a_pwm = GPIO.PWM(self.a_pwm_pin, 100) ; self.b_pwm = GPIO.PWM(self.b_pwm_pin, 100) ;
        self.a_pwm.start(0) ; self.b_pwm.start(0)

        # Stop the robot if moving initially
        self.stop()
        
        # Use ros node to listen to input
        if rosmode:
            rospy.init_node("robot", anonymous = True)
            # If keyboard teleop twist node is used
            if sys.argv[-1] == "keyboard":
                print("*** Listening for keyboard control.....")
                rospy.Subscriber("cmd_vel", Twist, self.kb_callback)
                rospy.spin()
            # If joy node is used
            elif sys.argv[-1] == "joy":
                print("*** Listening for joy node control...")
                rospy.Subscriber("joy", Joy, self.joy_callback )
                self.kfwd = 0 ; self.kleft = 0
                rate = rospy.Rate(5)
                while True:
                    if self.kfwd > 0 and self.kleft == 0 : self.forward()
                    elif self.kfwd > 0 and self.kleft == 1 : self.forward(25)
                    elif self.kfwd > 0 and self.kleft == -1 : self.forward(-25)
                    elif self.kfwd < 0 : self.reverse()
                    elif self.kleft == 1 : self.left()
                    elif self.kleft == -1 : self.right()
                    else : self.stop()

                    rate.sleep()
                    
    def joy_callback(self, resp):
        self.kfwd, self.kleft = resp.axes[4], resp.axes[6]
        
    def kb_callback(self, resp):
        if resp.linear.x > 0 and resp.angular.z == 0 : self.forward()
        elif resp.linear.x > 0 and resp.angular.z > 0 : self.forward(25)
        elif resp.linear.x > 0 and resp.angular.z < 0 : self.forward(-25)
        elif resp.linear.x < 0 : self.reverse()
        elif resp.angular.z > 0 : self.left()
        elif resp.angular.z < 0 : self.right()
        else : self.stop()
        
    def left(self):
        # Set direction of rotation
        GPIO.output(self.a1,GPIO.HIGH) ; GPIO.output(self.a2, GPIO.LOW)
        GPIO.output(self.b1,GPIO.LOW) ; GPIO.output(self.b2, GPIO.HIGH)

        # Set speed
        self.a_pwm.ChangeDutyCycle(self.wheel_omega_rot) ; self.b_pwm.ChangeDutyCycle(self.wheel_omega_rot) ;
        
    def right(self):
        # Set direction of rotation
        GPIO.output(self.a1,GPIO.LOW) ; GPIO.output(self.a2, GPIO.HIGH)
        GPIO.output(self.b1,GPIO.HIGH) ; GPIO.output(self.b2, GPIO.LOW)

        # Set speed
        self.a_pwm.ChangeDutyCycle(self.wheel_omega_rot) ; self.b_pwm.ChangeDutyCycle(self.wheel_omega_rot) ;

    def stop(self):
        # brake
        GPIO.output(self.a1,GPIO.LOW) ; GPIO.output(self.a2, GPIO.LOW)
        GPIO.output(self.b1,GPIO.LOW) ; GPIO.output(self.b2, GPIO.LOW)

    def forward(self , e = 0):
        # Set direction of rotation
        GPIO.output(self.a1,GPIO.LOW) ; GPIO.output(self.a2, GPIO.HIGH)
        GPIO.output(self.b1,GPIO.LOW) ; GPIO.output(self.b2, GPIO.HIGH)

        # Set speed
        if e == 0:
            self.a_pwm.ChangeDutyCycle(self.wheel_omega) ; self.b_pwm.ChangeDutyCycle(self.wheel_omega) ;
        elif e > 0:
            self.a_pwm.ChangeDutyCycle(self.wheel_omega - e) ; self.b_pwm.ChangeDutyCycle(self.wheel_omega) ;
        else :
            self.a_pwm.ChangeDutyCycle(self.wheel_omega) ; self.b_pwm.ChangeDutyCycle(self.wheel_omega + e) ;

    def reverse(self, e = 0):
        # Set direction of rotation
        GPIO.output(self.a1,GPIO.HIGH) ; GPIO.output(self.a2, GPIO.LOW)
        GPIO.output(self.b1,GPIO.HIGH) ; GPIO.output(self.b2, GPIO.LOW)

        # Set speed
        if e == 0:
            self.a_pwm.ChangeDutyCycle(self.wheel_omega) ; self.b_pwm.ChangeDutyCycle(self.wheel_omega) ;
        elif e > 0:
            self.a_pwm.ChangeDutyCycle(self.wheel_omega - e) ; self.b_pwm.ChangeDutyCycle(self.wheel_omega) ;
        else :
            self.a_pwm.ChangeDutyCycle(self.wheel_omega) ; self.b_pwm.ChangeDutyCycle(self.wheel_omega + e) ;
        
    
if __name__ == '__main__':
    a = Robot(True)

