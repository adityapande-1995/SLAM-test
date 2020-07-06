#!python
from __future__ import print_function
import RPi.GPIO as GPIO
import rospy
import time
import threading

# Odometry node

class Odo:
    def __init__(self, rosmode = False):
        print("Starting odometry node..")
        # Pinouts
        GPIO.setmode(GPIO.BCM)
        self.m1h1 = 1 ; self.m1h2 = 7 ; self.m2h1 = 25; self.m2h2 = 8 ;
        for p in [self.m1h1, self.m1h2, self.m2h1, self.m2h2 ] : GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Start monitoring frequencies
        self.m1f = 0 ; self.m2f = 0
        self.m1t = threading.Thread(target=self.monitor_m1)
        #self.m2t = threading.Thread(target=self.monitor_m2)
        self.m1t.start() ;
        #self.m2t.start()

        while True:
            print(self.m1f, self.m2f);
            time.sleep(0.1)

        self.m1t.join() ;
        #self.m2t.join()
        
    def monitor_m1(self):
        NUM_CYCLES = 10
        while True:
            start = time.time()
            for impulse_count in range(NUM_CYCLES): GPIO.wait_for_edge(self.m1h1 , GPIO.FALLING)
            duration = time.time() - start      #seconds to run for loop
            frequency = NUM_CYCLES / duration   #in Hz
            print("m1 freq updated")
            self.m1f = frequency

    def monitor_m2(self):
        NUM_CYCLES = 10
        while True:
            start = time.time()
            for impulse_count in range(NUM_CYCLES): GPIO.wait_for_edge(self.m2h1 , GPIO.FALLING)
            duration = time.time() - start      #seconds to run for loop
            frequency = NUM_CYCLES / duration   #in Hz
            print("m2 freq updated")
            self.m2f = frequency
            
            
if __name__ == "__main__":
    a = Odo()
    
            
