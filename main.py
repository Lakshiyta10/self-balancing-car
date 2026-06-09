# main.py - Self-Balancing Car with Raspberry Pi Pico
#
# Author: Gemini
# Date: 2025-09-08
#
# A structured, intermediate-level code for a self-balancing robot.
# Uses classes for MPU6050, Motor Driver, and PID controller.

import machine
import utime
import math

# -----------------------------------------------------------------------------
# MPU6050 IMU Class (I2C Communication and Angle Calculation)
# -----------------------------------------------------------------------------
class MPU6050:
    """
    A class to interact with the MPU-6050 IMU.
    Handles reading accelerometer/gyroscope data and calculating the angle
    using a complementary filter.
    """
    # Register addresses
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    PWR_MGMT_1 = 0x6B
    
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        self.alpha = 0.98  # Complementary filter constant
        self.angle = 0.0
        self._wake()

    def _wake(self):
        """Wakes the MPU-6050 from sleep mode."""
        self.i2c.writeto_mem(self.addr, self.PWR_MGMT_1, b'\x00')

    def _read_raw_word(self, reg):
        """Reads a 16-bit signed value from two 8-bit registers."""
        high = self.i2c.readfrom_mem(self.addr, reg, 1)[0]
        low = self.i2c.readfrom_mem(self.addr, reg + 1, 1)[0]
        val = (high << 8) | low
        # Convert to signed value
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def get_values(self):
        """Returns accelerometer and gyroscope values."""
        accel_x = self._read_raw_word(self.ACCEL_XOUT_H)
        accel_y = self._read_raw_word(self.ACCEL_XOUT_H + 2)
        accel_z = self._read_raw_word(self.ACCEL_XOUT_H + 4)
        gyro_x = self._read_raw_word(self.GYRO_XOUT_H)
        
        return {'accel_x': accel_x, 'accel_y': accel_y, 'accel_z': accel_z, 'gyro_x': gyro_x}

    def update_angle(self, dt):
        """
        Calculates the current angle using a complementary filter.
        dt (delta time) is crucial for accurate integration.
        """
        vals = self.get_values()
        
        # Calculate angle from accelerometer (long-term, stable)
        # Using atan2 is more robust than atan
        accel_angle = math.degrees(math.atan2(vals['accel_y'], vals['accel_z']))
        
        # Gyroscope rate (short-term, responsive)
        # Gyro_x is the rotation around the X-axis (the wheel axle)
        # Sensitivity factor for +/- 250 deg/s is 131
        gyro_rate = vals['gyro_x'] / 131.0
        
        # Complementary filter
        self.angle = self.alpha * (self.angle + gyro_rate * dt) + (1 - self.alpha) * accel_angle
        return self.angle

# -----------------------------------------------------------------------------
# PID Controller Class
# -----------------------------------------------------------------------------
class PID:
    """A simple PID controller."""
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self._last_error = 0
        self._integral = 0

    def compute(self, current_value, dt):
        """Calculates the PID output value for a given input."""
        error = self.setpoint - current_value
        self._integral += error * dt
        derivative = (error - self._last_error) / dt
        
        output = (self.Kp * error) + (self.Ki * self._integral) + (self.Kd * derivative)
        
        self._last_error = error
        return output
    
    def set_gains(self, Kp, Ki, Kd):
        """Allows for runtime tuning of PID gains."""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

# -----------------------------------------------------------------------------
# Motor Driver Class (for L298N)
# -----------------------------------------------------------------------------
class MotorDriver:
    """Controls two motors using an L298N H-Bridge driver."""
    def __init__(self, enA_pin, in1_pin, in2_pin, enB_pin, in3_pin, in4_pin):
        # Left Motor
        self.pwm_a = machine.PWM(machine.Pin(enA_pin))
        self.in1 = machine.Pin(in1_pin, machine.Pin.OUT)
        self.in2 = machine.Pin(in2_pin, machine.Pin.OUT)
        # Right Motor
        self.pwm_b = machine.PWM(machine.Pin(enB_pin))
        self.in3 = machine.Pin(in3_pin, machine.Pin.OUT)
        self.in4 = machine.Pin(in4_pin, machine.Pin.OUT)
        
        # Set PWM frequency
        self.pwm_a.freq(1000)
        self.pwm_b.freq(1000)

    def set_speeds(self, left_speed, right_speed):
        """
        Sets the speed and direction of the motors.
        Speed is a value from -100 to 100.
        """
        # Control Left Motor
        self._set_motor_speed(self.pwm_a, self.in1, self.in2, left_speed)
        # Control Right Motor
        self._set_motor_speed(self.pwm_b, self.in3, self.in4, right_speed)

    def _set_motor_speed(self, pwm, in_a, in_b, speed):
        # Clamp speed to the range -100 to 100
        speed = max(-100, min(100, speed))
        
        # Convert speed to 16-bit duty cycle
        duty_cycle = int(abs(speed) / 100 * 65535)
        
        if speed > 0:  # Forward
            in_a.high()
            in_b.low()
        elif speed < 0:  # Backward
            in_a.low()
            in_b.high()
        else:  # Stop
            in_a.low()
            in_b.low()
            
        pwm.duty_u16(duty_cycle)

    def stop(self):
        """Stops both motors."""
        self.set_speeds(0, 0)

# -----------------------------------------------------------------------------
# Main Program
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    
    # --- HARDWARE CONFIGURATION ---
    # I2C for MPU6050
    I2C_SDA_PIN = machine.Pin(0)
    I2C_SCL_PIN = machine.Pin(1)
    
    # L298N Motor Driver Pins
    L298N_ENA = 15
    L298N_IN1 = 13
    L298N_IN2 = 12
    L298N_ENB = 10
    L298N_IN3 = 9
    L298N_IN4 = 8

    # --- PID TUNING ---
    # These values require careful tuning! Start with Ki and Kd as 0.
    KP = 25.0
    KI = 1.5
    KD = 0.8
    
    # The desired angle (0 degrees for vertical)
    # Note: You may need a small offset based on your robot's center of mass.
    ANGLE_SETPOINT = 0.0 
    
    # --- INITIALIZATION ---
    print("Initializing components...")
    i2c = machine.I2C(0, sda=I2C_SDA_PIN, scl=I2C_SCL_PIN, freq=400000)
    imu = MPU6050(i2c)
    motors = MotorDriver(L298N_ENA, L298N_IN1, L298N_IN2, L298N_ENB, L298N_IN3, L298N_IN4)
    pid = PID(KP, KI, KD, setpoint=ANGLE_SETPOINT)
    
    # Timing variables for calculating dt
    last_time = utime.ticks_us()
    
    print("Initialization complete. Starting balancing loop...")
    
    try:
        while True:
            # Calculate delta time (dt) in seconds
            current_time = utime.ticks_us()
            dt = utime.ticks_diff(current_time, last_time) / 1_000_000.0
            last_time = current_time
            
            # 1. SENSE: Get the current angle from the IMU
            current_angle = imu.update_angle(dt)
            
            # 2. CONTROL: Compute the PID output
            pid_output = pid.compute(current_angle, dt)
            
            # 3. ACTUATE: Set motor speeds
            # Safety check: If the robot has fallen, stop the motors
            if abs(current_angle) > 35:
                motors.stop()
            else:
                motors.set_speeds(pid_output, pid_output)
            
            # Debugging print statement (optional)
            print(f"Angle: {current_angle:.2f} | PID Output: {pid_output:.2f}")
            
            # Small delay to prevent overwhelming the processor
            utime.sleep_ms(10)

    except KeyboardInterrupt:
        print("Program stopped by user.")
    finally:
        # Ensure motors are stopped on exit
        motors.stop()
        print("Motors stopped. Goodbye.")
