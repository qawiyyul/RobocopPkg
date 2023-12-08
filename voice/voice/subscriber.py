import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import gtts
from playsound import playsound

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            '/humans',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(5.0, self.timer_callback)        
        self.num=0
        self.count=0
    def listener_callback(self, msg):
        print('int received', msg.data)
        self.num=msg.data
    
    def timer_callback(self):
        print('timer:', self.num)
        if self.num>0:
            self.count +=1
            if self.num<=2:
               if self.num == 1:
                   playsound("1.mp3")
               else:
                   playsound("2.mp3")
            else:
                playsound("many.mp3")
            
#            speech = gtts.gTTS("Human found!")
#            engage = gtts.gTTS("permission to engage?")
#            one = gtts.gTTS("1")
#            two = gtts.gTTS("2")
#            many = gtts.gTTS("many")
#            sirstopsir = gtts.gTTS("SIR STOP SIR!")
#            speech.save("human.mp3")
#            engage.save("engage.mp3")
#            one.save("1.mp3")
#            two.save("2.mp3")
#            many.save("many.mp3")
#            sirstopsir.save("sirstopsir.mp3")
            playsound("human.mp3")
            if self.count%3==0:
                playsound("engage.mp3")
            elif self.count%7==0:
                playsound("sirstopsir.mp3")
    

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
