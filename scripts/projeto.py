#!/usr/bin/env python3
import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
from tf import transformations
from utils.distance_to_box import calculate_distance_to_box
from utils.distance_to_target import is_close_to_target_box
import json

# Estados do robo
segue_linha = 0
slalon = 1
pega_creeper = 2
# box_found = 3
mapeia_creepers = 4
rotaciona_direcao_creeper = 5
anda_direcao_creeper = 6
centraliza_creeper = 7
# volta_caixa = 8
delivery_creeper= 9
back_to_beginning = 10

class Projeto:
    def __init__(self):
        # Definição do estado
        self.state = mapeia_creepers

        # Variáveis de controle
        # self.blue_box_reached = False
        self.first_curve_done = False
        self.move_forward_done = False
        self.second_curve_done = False
        # self.box_found = False
        # self.blue_box_direction = False
        # self.close_to_blue_box = False
        self.image_raw = None
        self.creeper_direction_found = False

        # Creepers
        self.creeper_data = {}
        self.angle_before_mapping = None
        self.first_angle_before_mapping_captured = False
        self.creeper_delivered = False

        #Definição do objetivo do robô
        # self.goal = ("blue", 12, "horse")
        
        # Encontrar as caixas na mobilenet
        # self.box_positions = {}
        # self.horse_found = False
        # self.bird_found = False
        # self.bike_found = False
        # self.return_to_track = False
        # self.angle_before_box = None
        # self.start_find_box_state = False
        # self.distance_to_bifurcation = None
        # self.bifurcation_coords = None
        # self.mobilenet_payload = None
        # self.odom_sub = rospy.Subscriber('/odom', Odometry, self.get_odom)
        # self.mobilenet_sub = rospy.Subscriber('/mobilenet_results', String, self.mobilenet_data_callback)
        # rospy.on_shutdown(self.shutdown)

        #Estados do creeper
        self.coordenadas_iniciais_x = None
        self.coordenadas_iniciais_y = None
        self.coordenadas_iniciais_salvas = False

        # Estados da garra
        # self.levantou_garra = False
        # self.fechou_garra = False
        # self.pegou_creeper = False
        # self.gira_direcao_pista = False
        # self.voltou_pista = False
        # self.origin_direction_found = False

        # self.area_caixa_azul = 0
        # self.area_caixa_vermelha = 0
        # self.previous_blue_box_area = 0
        self.bridge = CvBridge()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                             Twist, 
                                             queue_size=1)
        self.img_data_sub = rospy.Subscriber('/img_data', String, self.decode_img_data, queue_size=1)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        # self.ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        # self.garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)
        self.creeper_data_subscriber = rospy.Subscriber('/creeper_data', String, self.creeper_data_callback)
        self.twist = Twist()
        self.laser_msg = -1
        rospy.wait_for_message('/img_data', String)
        rospy.wait_for_message("/scan", LaserScan)
        # rospy.wait_for_message("/mobilenet_results", String)
        rospy.wait_for_message("/creeper_data", String)
        distances = self.laser_msg.ranges[::-1]
        for i, distance in enumerate(distances):
            if not np.isinf(distance):
                self.distance_to_bifurcation = distance*math.cos(math.radians(i+1)) - 0.1
                (x, y, _) = self.position
                self.bifurcation_coords = (x, y-self.distance_to_bifurcation)
                break

        self.cx = -1
        # self.cx_blue = -1
        # self.cy_blue = -1
        # self.cx_blue_creeper = -1
        # self.cy_blue_creeper = -1
        self.cx_green = -1
        self.cy_green = -1
        # self.cx_pink = -1
        # self.cy_pink = -1
        self.cy = -1
        self.h = -1
        self.w = -1

        self.hertz = 250
        self.rate = rospy.Rate(self.hertz)


    def shutdown(self):
        '''
        Para o robô quando o node é interrompido
        '''
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())

    def laser_callback(self, msg):
        '''
        Recebe as leituras do LIDAR e salva na variável self.laser_msg
        Responsável também por ativar o estado de mapeamento das bases
        '''
        self.laser_msg = msg
        if not np.isinf(msg.ranges[83:92]).any() and \
        (all(x < 1 for x in msg.ranges[83:92])) and \
            (all(x > 2 for x in msg.ranges[175:185])) and \
            (not self.horse_found or not self.bird_found or not self.bike_found) \
            and self.state != slalon \
            and self.state != rotaciona_direcao_creeper\
            and self.state != anda_direcao_creeper\
            and self.state != centraliza_creeper\
            and self.state != pega_creeper\
            and not self.creeper_delivered \
            # and calculate_distance_to_box(self.position, self.box_positions) > 0.7:
            # self.box_found = True
            # self.state = box_found
        elif not np.isinf(msg.ranges[83:92]).any() and \
        (all(x < 1 for x in msg.ranges[83:92])) and \
            (all(x > 2 for x in msg.ranges[175:185])) \
            and not self.creeper_delivered \
            and self.state != slalon \
            and self.state != rotaciona_direcao_creeper\
            and self.state != anda_direcao_creeper\
            and self.state != centraliza_creeper\
            and not self.creeper_delivered \
            and self.state != pega_creeper:
                self.box_found = True


    # def mobilenet_data_callback(self, msg):
    #     '''
    #     Recebe os dados do node da mobilenet 
    #     '''
    #     if msg.data is not None and msg.data != "":
    #         self.mobilenet_payload = json.loads(msg.data)

    def segue_linha_amarela(self):
        '''
        Segue a linha amarela da pista
        '''
        err = self.cx - self.w/2
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 500

        # if self.area_caixa_vermelha > 70000:
        #         self.state = slalon

        if self.creeper_delivered:
            (x,y,_) = self.position
            dist = math.sqrt((x - self.bifurcation_coords[0])**2 + (y - self.bifurcation_coords[1])**2)
            if dist < 0.55:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                self.state = rotaciona_direcao_creeper
                rospy.signal_shutdown("Objetivo concluído")

        # if (self.bird_found and self.bike_found) and not self.voltou_pista:
        #     (x,y,_) = self.position
        #     dist = math.sqrt((x - self.bifurcation_coords[0])**2 + (y - self.bifurcation_coords[1])**2)
        #     if dist < 0.55:
        #         self.twist.linear.x = 0.0
        #         self.twist.angular.z = 0.0
        #         self.cmd_vel_pub.publish(self.twist)
        #         self.state = rotaciona_direcao_creeper

        target_box = self.goal[2]

        if self.voltou_pista and is_close_to_target_box(self.position, self.box_positions, target_box) and self.pegou_creeper and \
            not np.isinf(self.laser_msg.ranges[83:92]).any() and \
            (all(x < 1 for x in self.laser_msg.ranges[83:92])) and \
            (all(x > 2 for x in self.laser_msg.ranges[175:185])) and \
            not self.creeper_delivered:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            self.state = delivery_creeper 
        # self.twist = Twist()
        self.cmd_vel_pub.publish(self.twist)
        self.rate.sleep()
    
    # def faz_slalon(self):
    #     '''
    #     Faz o slalon com controles proporcionais baseados no LiDAR e nas áreas das caixas
    #     '''
    #     err = self.cx_blue - self.w/2
    #     self.twist.angular.z = - float(err) / 300
    #     if self.area_caixa_azul < 1000:
    #         self.twist.angular.z = 0.7
    #     if self.area_caixa_azul > 40000 and self.laser_msg.ranges[0] < 0.6:
    #         self.blue_box_direction = True
        
    #     if self.area_caixa_azul < 10 and self.blue_box_direction:
    #         self.blue_box_reached = True

    #     if self.blue_box_direction and (self.laser_msg.ranges[0] < 0.6 or self.blue_box_direction) \
    #         and not self.blue_box_reached:
    #         self.twist.angular.z = - (self.area_caixa_azul / (100*1000))

    #     if self.area_caixa_azul == 0 and self.blue_box_direction:
    #         self.twist.angular.z = 0
        
    #     laser_np = np.array(self.laser_msg.ranges[40:45])
    #     laser_np = laser_np[~np.isinf(laser_np)] # Remove inf
    #     if self.blue_box_reached and len(laser_np[laser_np > 2]) >= 3:
    #         self.first_curve_done = True
    
    #     if self.first_curve_done:
    #         self.twist.angular.z = 0.5

    #     if self.first_curve_done and self.laser_msg.ranges[0] > 1:
    #         self.blue_box_reached = False
    #         self.first_curve_done = False
    #         self.move_forward_done = False
    #         self.second_curve_done = False
    #         self.box_found = False
    #         self.blue_box_direction = False
    #         self.close_to_blue_box = False
    #         self.state = segue_linha

    #     self.twist.linear.x = 0.2

    #     self.cmd_vel_pub.publish(self.twist)
    #     self.rate.sleep()
    
    # def mapeia_bases(self):
    #     '''
    #     Mapeia as bases da pista de acordo com suas classes da Mobilenet
    #     guardando suas coordenadas em um dicionário para a entrega do creeper posteriormente
    #     '''
    #     self.twist.linear.x = 0.0
    #     self.twist.angular.z = 0.0
    #     if not self.start_find_box_state:
    #         self.angle_before_box = self.position[2]
    #         self.start_find_box_state = True
    #     self.cmd_vel_pub.publish(self.twist)
    #     # Rotaciona o robo para a direita até ficar no centro
    #     self.twist.angular.z = 0.5
    #     x, y, angle_robot = self.position
    #     if self.mobilenet_payload['x_center'] is not None and self.box_found:
    #         err_rot = self.mobilenet_payload['x_center'] - self.w/2
    #         self.twist.angular.z = -float(err_rot) / 500
    #         if abs(err_rot) < 30:
    #             index_min_distance = np.argmin(self.laser_msg.ranges)
    #             distance_to_box = self.laser_msg.ranges[index_min_distance]
    #             x_box = x
    #             if angle_robot > 0:
    #                 y_box = y + distance_to_box
    #             else:
    #                 y_box = y - distance_to_box
    #             if self.mobilenet_payload['classe'] == 'horse':
    #                 self.box_positions['horse'] = (x_box, y_box)
    #                 self.horse_found = True
    #                 self.return_to_track = True
    #                 self.box_found = False
    #             elif self.mobilenet_payload['classe'] == 'bird':
    #                 self.box_positions['bird'] = (x_box, y_box)
    #                 self.bird_found = True
    #                 self.box_found = False
    #             elif self.mobilenet_payload['classe'] == 'bicycle':
    #                 self.box_positions['bicycle'] = (x_box, y_box)
    #                 self.bike_found = True
    #                 self.box_found = False
    #                 self.return_to_track = True
                
    #     # Rotaciona o robô de volta para pista
    #     if self.return_to_track:
    #         while not np.isclose(self.position[2], self.angle_before_box, atol=0.1):
    #             self.twist.angular.z = -0.3
    #             if self.bike_found:
    #                 self.twist.angular.z = 0.3
    #             # self.twist.linear.x = 0.04
    #             self.cmd_vel_pub.publish(self.twist)
    #             self.rate.sleep()
    #         self.state = segue_linha
    #         self.return_to_track = False
    #         self.start_find_box_state = False

    #     self.cmd_vel_pub.publish(self.twist)
    #     self.last_angle = angle_robot

    def map_creepers(self):
        '''
        Mapeia os creepers guardando sua cor, seu ID e suas coordenadas em um dicionário para que o robô
        pegue o creeper desejado posteriormente.
        '''
        if not self.first_angle_before_mapping_captured:
            self.angle_before_mapping = self.position[2]
            self.first_angle_before_mapping_captured = True
        while len(self.creepers_data.keys()) < 6:
            # Rotaciona o robô com w -0.2
            self.twist.angular.z = -0.2
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()
        # Voltar para o ângulo original
        while not np.isclose(self.position[2], self.angle_before_mapping, atol=0.1):
            self.twist.angular.z = 0.3
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()
        self.state = segue_linha
    
    def rotaciona_creeper(self):
        '''
        Rotaciona o robô na direção do creeper desejado a partir das coordenadas mapeadas anteriormente
        '''
        if not(self.coordenadas_iniciais_salvas):
            self.coordenadas_iniciais_x = self.position[0]
            self.coordenadas_iniciais_y = self.position[1]
            self.coordenadas_iniciais_salvas = True
        else:
            self.garra.publish(0)
            self.ombro.publish(-1)
            target_creeper_key = f"{self.goal[0]}{self.goal[1]}"
            creeper_data = self.creepers_data[target_creeper_key]
            angle_to_goal = math.atan2((creeper_data['y']  - self.position[1]), (creeper_data['x'] - self.position[0]))
            if abs(angle_to_goal - self.position[2]) > 0.1:
                self.twist.linear.x = 0.0
                self.twist.angular.z = -0.15
                self.cmd_vel_pub.publish(self.twist)
            else:
                self.state = anda_direcao_creeper

    def segue_creeper(self):
        '''
        Faz o robô andar na direção do creeper desejado até uma distância mínima de 0.5m a partir das coordenadas mapeadas
        anteriormente, depois ativa a centralização por cor do creeper
        '''
        if not(self.creeper_direction_found):
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            self.creeper_direction_found = True
        
        else:
            target_creeper_key = f"{self.goal[0]}{self.goal[1]}"
            creeper_data = self.creepers_data[target_creeper_key]
            dist = math.sqrt((creeper_data['x'] - self.position[0])**2 + (creeper_data['y'] - self.position[1])**2)
            while dist > 0.5:
                self.twist.linear.x = 0.15
                self.twist.angular.z = 0.0
                self.rate.sleep()   
                self.cmd_vel_pub.publish(self.twist)
                dist= math.sqrt((creeper_data['x'] - self.position[0])**2 + (creeper_data['y'] - self.position[1])**2)
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            self.state = centraliza_creeper

    def centraliza_cor_creeper(self):
        '''
        Centraliza o robô na cor do creeper desejado a partir da cor mapeada anteriormente e faz o robô se aproximar
        até uma distância mínima de 0.4m do creeper a partir do LiDAR e depois ativa a captura do creeper
        '''
        cx_goal, _ = self.centro_cor()
        err = cx_goal - self.w/2
        self.twist.angular.z = - float(err) / 300
        self.cmd_vel_pub.publish(self.twist)

        if (any(x < 0.4 for x in self.laser_msg.ranges[0:5])) or (any(x < 0.4 for x in self.laser_msg.ranges[356:359])):
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(1.0)
            self.state = pega_creeper
            
        else:
            self.twist.linear.x = 0.05
            self.twist.angular.z = - float(err) / 300
            self.cmd_vel_pub.publish(self.twist)

    def pega_creeper_garra(self):
        '''
        Faz o robô pegar o creeper com a garra e retornar para as coordenadas que ele estava antes de pegar o creeper
        '''
        if not(self.levantou_garra):
            self.garra.publish(-1)
            rospy.sleep(0.5)
            self.ombro.publish(0)
            rospy.sleep(0.5)
            self.twist.linear.x = 0.1
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(2.2)
            self.cmd_vel_pub.publish(Twist())
            rospy.sleep(1.0)
            self.levantou_garra = True
        elif not(self.fechou_garra):
            self.garra.publish(1)
            rospy.sleep(1.0)
            self.fechou_garra = True
        elif not(self.pegou_creeper):
            self.ombro.publish(1.5)
            rospy.sleep(1.0)
            self.pegou_creeper = True
        elif not(self.gira_direcao_pista):
            angle_to_goal = math.atan2((self.coordenadas_iniciais_y  - self.position[1]), (self.coordenadas_iniciais_x - self.position[0]))
            if abs(angle_to_goal - self.position[2]) > 0.1:
                self.twist.linear.x = 0.0
                self.twist.angular.z = -0.5
                self.cmd_vel_pub.publish(self.twist)
            else:
                self.gira_direcao_pista = True
        elif not(self.voltou_pista):
            if not(self.origin_direction_found):
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                self.origin_direction_found = True
            
            else:
                dist = math.sqrt((self.coordenadas_iniciais_x  - self.position[0])**2 + (self.coordenadas_iniciais_y  - self.position[1])**2)
                while dist > 0.3:
                    self.twist.linear.x = 0.15
                    self.twist.angular.z = 0.0
                    self.rate.sleep()   
                    self.cmd_vel_pub.publish(self.twist)
                    dist = math.sqrt((self.coordenadas_iniciais_x  - self.position[0])**2 + (self.coordenadas_iniciais_y  - self.position[1])**2)
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                self.voltou_pista = True
                self.state = segue_linha

    # def retorna_para_base(self):
    #     '''
    #     Faz o robô voltar a seguir pela pista pelo caminho mais próximo até a base desejada
    #     '''
    #     goal_box = self.goal[2]
    #     if goal_box == 'horse':
    #         self.twist.angular.z = -0.7
    #         self.cmd_vel_pub.publish(self.twist)
    #         rospy.sleep(3.8)
    #         self.twist.angular.z = 0.0
    #         self.cmd_vel_pub.publish(self.twist)
    #         rospy.sleep(1.0)
    #         self.state = segue_linha
    #     elif goal_box == 'bird' or goal_box == 'bicycle':
    #         self.state = segue_linha

    def entrega_creeper(self):
        '''
        Para na base desejada, avança até chegar mais próximo e abre a garra, assim entregando o creeper
        '''
        self.twist.angular.z = 0.5
        
        if self.mobilenet_payload['x_center'] is not None:
            err_rot = self.mobilenet_payload['x_center'] - self.w/2
            self.twist.angular.z = -float(err_rot) / 500
            if abs(err_rot) < 30:
                #! VAI PRA FRENTE
                self.twist.linear.x = 0.1
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(2)
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(1)
                self.ombro.publish(0)
                rospy.sleep(1)
                self.garra.publish(-1)
                rospy.sleep(1)
                self.garra.publish(0)
                rospy.sleep(0.5)
                self.ombro.publish(-1)
                self.twist = Twist()
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(1)
                self.state = back_to_beginning
                self.creeper_delivered = True
        self.cmd_vel_pub.publish(self.twist)

    def retorna_para_origem(self):
        '''
         Faz o robô voltar a seguir pela pista pelo caminho mais próximo até a origem dependendo da base onde ele se encontra
        '''
        goal = self.goal[2]
        self.twist = Twist()    
        if goal == 'bicycle':
            self.twist.linear.x = -0.09
            self.twist.angular.z = 0.3
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(5)
            self.state = segue_linha
        else:
            self.twist.linear.x = -0.09
            self.twist.angular.z = -0.3
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(5)
            self.state = segue_linha

    def centro_cor(self):
        '''
        Define o centro que o robô vai procurar como objetivo dependendo da cor do creeper desejado 
        '''
        target_creeper_key = f"{self.goal[0]}{self.goal[1]}"
        creeper_data = self.creepers_data[target_creeper_key]
        color = creeper_data['cor']

        cx_goal = -1
        cy_goal = -1

        # if color == 'blue':
        #     cx_goal = self.cx_blue_creeper
        #     cy_goal = self.cy_blue_creeper
        # elif color == 'pink':
        #     cx_goal = self.cx_pink
        #     cy_goal = self.cy_pink
        if color == 'green':
            cx_goal = self.cx_green
            cy_goal = self.cy_green

        return cx_goal, cy_goal

    def get_odom(self, data):
        '''
        Recebe os dados do odômetro do robô
        '''
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        quat = data.pose.pose.orientation
        lista = [quat.x, quat.y, quat.z, quat.w]
        angulos_rad = transformations.euler_from_quaternion(lista)

        self.position = (x, y, angulos_rad[2])

    def decode_img_data(self, msg):
        '''
        Decodifica os dados do node de visão recebidos pelo robô
        '''
        data_decoded = json.loads(msg.data)
        self.cx = data_decoded['cx']
        self.cy = data_decoded['cy']
        self.w = data_decoded['w']
        self.h = data_decoded['h']
        # self.cx_blue = data_decoded['cx_blue']
        # self.cy_blue = data_decoded['cy_blue']
        # self.cx_blue_creeper = data_decoded['cx_blue_creeper']
        # self.cy_blue_creeper = data_decoded['cy_blue_creeper']
        self.cx_green = data_decoded['cx_green']
        self.cy_green = data_decoded['cy_green']
        # self.cx_pink = data_decoded['cx_pink']
        # self.cy_pink = data_decoded['cy_pink']
        # self.area_caixa_vermelha = data_decoded['red_box_area']
        # self.area_caixa_azul = data_decoded['blue_box_area']

    def creeper_data_callback(self, msg):
        '''
        Recebe os dados dos creeper do node de scan dos arucos
        '''
        if msg is not None:
            decoded_msg = json.loads(msg.data)
            self.creepers_data = decoded_msg
    
    def control(self):
        '''
        Função principal do robô, que controla o movimento do mesmo e ações a serem tomadas a partir do estado atual
        '''
        if self.state == segue_linha:
            self.segue_linha_amarela()
    
        if self.state == mapeia_creepers:
            self.map_creepers()

        if self.state == rotaciona_direcao_creeper:
            self.rotaciona_creeper()

        if self.state == anda_direcao_creeper:
            self.segue_creeper()

        if self.state == centraliza_creeper:
            self.centraliza_cor_creeper()

        if self.state == pega_creeper:
            self.pega_creeper_garra()

        # if self.state == volta_caixa:
        #     self.retorna_para_base() 
        
        if self.state == delivery_creeper:
            self.entrega_creeper()

        if self.state == back_to_beginning:
            self.retorna_para_origem()

if __name__=="__main__":
    rospy.init_node('follower')
    projeto_instance = Projeto()

    while not rospy.is_shutdown():
        projeto_instance.control()
    