#!/usr/bin/env python3

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.prev_time   = 0
   

    def output(self, error, time):
        
        dt = time - self.prev_time 
        if dt > 0:
            
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            self.prev_error = error 
            self.prev_time = time 
            
            return u 