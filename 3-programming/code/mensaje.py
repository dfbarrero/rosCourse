from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

vector = Vector3()
vector.x = 0
vector.y = 0
vector.z = 1
twist = Twist()
twist.linear = vector
twist.angular.x = 0
twist.angular.y = 0
twist.angular.z = 0
otroVector = Vector3(1, 0, 0)
