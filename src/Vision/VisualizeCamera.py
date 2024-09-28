import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

def image_callback(msg):
    # CompressedImage에서 데이터 디코딩
    np_arr = np.frombuffer(msg.data, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # 이미지 시각화
    cv2.imshow("Compressed Image", image)
    
    # 'q' 키를 눌렀는지 확인
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User requested shutdown.")

def main():
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber('/Image', CompressedImage, image_callback)

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
