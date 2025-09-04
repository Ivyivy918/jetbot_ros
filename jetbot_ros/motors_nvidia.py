import rclpy
from jetbot_ros.motors import MotorController
from Adafruit_MotorHAT import Adafruit_MotorHAT

class MotorControllerNV(MotorController):
    """
    Motor controller node that supports the original NVIDIA JetBot.
    Modified to auto-forward on startup for demonstration.
    """
    MOTOR_LEFT = 1      # left motor ID
    MOTOR_RIGHT = 2     # right motor ID
    
    def __init__(self):
        super().__init__()
        
        # open Adafruit MotorHAT driver
        self.driver = Adafruit_MotorHAT(i2c_bus=7)
        
        # get motor objects from driver
        self.motors = {
            self.MOTOR_LEFT : self.driver.getMotor(self.MOTOR_LEFT),
            self.MOTOR_RIGHT : self.driver.getMotor(self.MOTOR_RIGHT)
        }
        
        # 創建定時器讓馬達持續前進
        self.create_timer(0.1, self.auto_forward_callback)
        
    def auto_forward_callback(self):
        """定時器回調函數 - 讓馬達持續前進"""
        # 設定前進速度 (可調整 0.3 這個值，範圍 0.0-1.0)
        forward_speed = 0.5  # 50% 速度
        self.set_speed(forward_speed, forward_speed)
        
    def set_speed(self, left, right):
        """
        Sets the motor speeds between [-1.0, 1.0]
        """
        self._set_pwm(self.MOTOR_LEFT, left, self.left_trim)
        self._set_pwm(self.MOTOR_RIGHT, right, self.right_trim)
      
    def _set_pwm(self, motor, value, trim):
        # apply trim and convert [-1,1] to PWM value
        pwm = int(min(max((abs(value) + trim) * self.max_pwm, 0), self.max_pwm))
        self.motors[motor].setSpeed(pwm)
        
        # set the motor direction
        cmd = Adafruit_MotorHAT.RELEASE
        
        if value > 0:
            cmd = Adafruit_MotorHAT.FORWARD
        elif value < 0:
            cmd = Adafruit_MotorHAT.BACKWARD
            
        self.motors[motor].run(cmd)
    
    def destroy_node(self):
        self.set_speed(0.0, 0.0)
        super().destroy_node()
 
def main(args=None):
    rclpy.init(args=args)
    
    node = MotorControllerNV()
    print("Press Ctrl+C to stop")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()