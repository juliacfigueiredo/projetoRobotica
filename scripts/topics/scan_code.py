from cv2 import imshow
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry 
import cv2 as aruco
from tf import transformations
import tf
import json
import math



class ScanAruco():
  def __init__(self):
      self.bridge = CvBridge()
       #--- Define the aruco dictionary
      self.dic_creepers = {}
      self.aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
      self.parameters  = aruco.DetectorParameters_create()
      self.parameters.minDistanceToBorder = 0
      self.parameters.adaptiveThreshWinSizeMax = 1000

      self.position = None

      self.image_sub = rospy.Subscriber('/camera/image/compressed',
                                          CompressedImage, 
                                          self.image_callback, 
                                          queue_size=4, 
                                          buff_size = 2**24)
      self.odom_sub = rospy.Subscriber('/odom', Odometry, self.get_odom)
      self.creeper_data_pub = rospy.Publisher('/creeper_data', String, queue_size=10)

      rospy.wait_for_message('/odom', Odometry)


      self.loop_rate = rospy.Rate(1)
      self.img_pub = rospy.Publisher('/cam_vision', Image, queue_size=1)
      self.img_data_pub = rospy.Publisher('/img_data', String, queue_size=1)
      self.hertz = 250
      self.rate = rospy.Rate(self.hertz)
      self.id_to_find  = 200
      self.marker_size  = 20 #- [cm]
      
      
      self.calib_path  = "../../src/aruco/"
      self.camera_matrix   = np.loadtxt(self.calib_path+'cameraMatrix_raspi.txt', delimiter=',')
      self.camera_distortion   = np.loadtxt(self.calib_path+'cameraDistortion_raspi.txt', delimiter=',')

      self.font = cv2.FONT_HERSHEY_PLAIN
      self.scan_dist = 0
      self.contagem = 0
      self.mapeou = False
      self.position = None
  def get_odom(self, data):
      x = data.pose.pose.position.x
      y = data.pose.pose.position.y

      self.quat = data.pose.pose.orientation
      lista = [self.quat.x, self.quat.y, self.quat.z, self.quat.w]
      angulos_rad = transformations.euler_from_quaternion(lista)

      self.position = (x, y, angulos_rad[2])

  def image_callback(self, imagem):
    try:
      cv_image = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8") # imagem compressed
      compressed_image = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8") # imagem compressed
      gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
      corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
    


      if ids is not None:
        for i in range(0, len(ids)):
            
          ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.camera_distortion)
          rvec, tvec = ret[0][i,0,:], ret[1][i,0,:]
          
          

        
          distance = np.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
          distancenp = np.linalg.norm(tvec)


          
          
          FOCAL_LENGTH = 3.6 #3.04

          m = (self.camera_matrix[0][0]/FOCAL_LENGTH + self.camera_matrix[1][1]/FOCAL_LENGTH)/2
      
      
          pixel_length1 = math.sqrt(math.pow(corners[i][0][0][0] - corners[i][0][1][0], 2) + math.pow(corners[i][0][0][1] - corners[i][0][1][1], 2))
          pixel_length2 = math.sqrt(math.pow(corners[i][0][2][0] - corners[i][0][3][0], 2) + math.pow(corners[i][0][2][1] - corners[i][0][3][1], 2))
          pixlength = (pixel_length1+pixel_length2)/2
          dist = self.marker_size * FOCAL_LENGTH / (pixlength/m)

          m = self.marker_size/2
          pts = np.float32([[-m,m,m], [-m,-m,m], [m,-m,m], [m,m,m],[-m,m,0], [-m,-m,0], [m,-m,0], [m,m,0]])
          imgpts, _ = cv2.projectPoints(pts, rvec, tvec, self.camera_matrix, self.camera_distortion)
          imgpts = np.int32(imgpts).reshape(-1,2)
          cv_image = cv2.drawContours(cv_image, [imgpts[:4]],-1,(0,0,255),4)
          a,b,c,d = (imgpts[:4])
          color = self.detecta_cor_aruco(a,b,c,d,compressed_image)
          if color is not None:
            chave = color + str(int(ids[i]))
            if chave not in self.dic_creepers.keys():
                x_creeper, y_creeper = self.distancia_robo_para_posicao(tvec)
                self.dic_creepers[chave] = {'cor':color, 'id':int(ids[i]), 'x':x_creeper, 'y':y_creeper}
            self.contagem = (len(self.dic_creepers))
    
            if self.contagem == 6:
                self.mapeou = True

    
      
      # Exibe tela
      cv2.imshow("Camera", cv_image)
      cv2.waitKey(1)
    except CvBridgeError as e:
      print('ex', e)


  def distancia_robo_para_posicao(self, tvec):
      x, y, angle_robot = self.position
      tvec = [tvec[2], tvec[1], tvec[0]]
      tvec = np.array(tvec)/1000
      x_creeper, y_creeper, z_creeper = self.transforma_coordenadas(tvec)
      x_creeper_w = x_creeper + x
      y_creeper_w = y_creeper + y
      return x_creeper_w - 0.4834, y_creeper_w - 0.124

  def transforma_coordenadas(self, tvec):
      v1 = tf.transformations.unit_vector(tvec)
      q1 = [self.quat.x, self.quat.y, self.quat.z, self.quat.w]
      q2 = list(v1)
      q2.append(0.0)
      tvec_world = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q1, q2), tf.transformations.quaternion_conjugate(q1))[:3]
      return tvec_world
  
  def detecta_cor_aruco(self,a,b,c,d, image):


      #Cria retangulo ao redor do corpo do creeper a partir das coordenadas do aruco
      b =[b[0], b[1]+80]
      c =[c[0], c[1]+80]
      caixa = [a,b,c,d]
      color = None

      #Cria mascaras para detectar a cor do creeper
      hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
      lower_green= np.array([100/2, 235, 235],dtype=np.uint8)
      upper_green = np.array([140/2, 255, 255],dtype=np.uint8)
      mask_green = cv2.inRange(hsv, lower_green, upper_green)

      lower_pink = np.array([313/2, 235, 235],dtype=np.uint8)
      upper_pink = np.array([353/2, 255, 255],dtype=np.uint8)
      mask_pink = cv2.inRange(hsv, lower_pink, upper_pink)

      lower_blue = np.array([219/2, 235, 235],dtype=np.uint8)
      upper_blue = np.array([259/2, 255, 255],dtype=np.uint8)
      mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

      #Verifica se a cor do creeper esta dentro da mascara
      for i in range(a[0], d[0]):
          for j in range(a[1], c[1]):
              if mask_green[j][i] == 255:
                  color = 'green'
                  break
              elif mask_pink[j][i] == 255:
                  color = 'pink'
                  break
              elif mask_blue[j][i] == 255:
                  color = 'blue'
                  break
      
      #caixa = np.int32(caixa).reshape(-1,2)
      #cv_image = cv2.drawContours(cv_image, [caixa],-1,(0,0,255),4)
      return color


  def start(self):
    while not rospy.is_shutdown():
      encoded_data = json.dumps(self.dic_creepers)
      self.creeper_data_pub.publish(encoded_data)
      self.rate.sleep()

if __name__ == '__main__':
  rospy.init_node('fireworks_aruco', anonymous=True)
  mynode = ScanAruco()
  mynode.start()
