import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty, signal

class TeleopNode(Node):
    def __init__(self):
        super().__init__('manual_teleop')
        self.publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.cmd = Twist()
        self.speed = 0.5  # Default linear speed
        self.turn = 0.5   # Default angular speed

        # Store terminal settings for restoration
        self.settings = termios.tcgetattr(sys.stdin)

        # Register signal handler for proper exit
        signal.signal(signal.SIGINT, self.handle_exit)

        # Print instructions
        self.print_instructions()

    def get_key(self):
        """Capture key presses in a non-blocking way."""
        try:
            tty.setraw(sys.stdin.fileno())
            select.select([sys.stdin], [], [], 0)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def print_instructions(self):
        print("""
        Manual Teleop Control:
        ---------------------------
        W : Move Forward
        S : Move Backward
        A : Turn Left
        D : Turn Right
        X : Stop

        Q/Z : Increase/Decrease Speed
        E/C : Increase/Decrease Turn Rate
        CTRL+C to exit
        """)

    def handle_exit(self, signum=None, frame=None):
        """Handle clean exit with CTRL+C by stopping the robot and restoring terminal settings."""
        print("\nExiting Teleop... Stopping robot.")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher.publish(self.cmd)
        
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        # Shutdown ROS 2
        rclpy.shutdown()
        sys.exit(0)

    def run(self):
        """Listen for key inputs and update the Twist message accordingly."""
        try:
            while True:
                key = self.get_key()
                
                if key == 'w':
                    self.cmd.linear.x = -self.speed
                    self.cmd.angular.z = 0.0
                elif key == 's':
                    self.cmd.linear.x = self.speed
                    self.cmd.angular.z = 0.0
                elif key == 'a':
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = -self.turn
                elif key == 'd':
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = self.turn
                elif key == 'x':
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = 0.0
                elif key == 'q':
                    self.speed += 0.2
                    print(f"Speed increased: {self.speed}")
                elif key == 'z':
                    self.speed = max(0.2, self.speed - 0.2)
                    print(f"Speed decreased: {self.speed}")
                elif key == 'e':
                    self.turn += 0.2
                    print(f"Turn rate increased: {self.turn}")
                elif key == 'c':
                    self.turn = max(0.2, self.turn - 0.2)
                    print(f"Turn rate decreased: {self.turn}")
                elif key == '\x03':  # CTRL+C
                    self.handle_exit()
                else:
                    continue
                
                # Publish new command
                self.publisher.publish(self.cmd)

        except KeyboardInterrupt:
            self.handle_exit()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
