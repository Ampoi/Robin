import rclpy
import subprocess

process = subprocess.Popen(
  ["bun", "run", "dev"],
  stdout=subprocess.PIPE,
  stderr=subprocess.PIPE,
  cwd="../client",
  text=True
)

class Client(Node):
  def __init__(self):
    super().__init__('client')
    print("Client Node Initialized")
    try:
      for line in process.stdout:
        print(line, end="")
    except KeyboardInterrupt:
      process.terminate()
      print("\nProcess terminated.")

def main(args=None):
  rclpy.init(args=args)
  
  client = Client()
  rclpy.spin(client)
  
  client.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()