import rospy
from std_msgs.msg import(
    String
)

if __name__ == "__main__":
    rospy.init_node("test")

    pub = rospy.Publisher('t', String, queue_size=1)

    msg = String()
    msg.data = "Hello"
    
    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.sleep(1)
