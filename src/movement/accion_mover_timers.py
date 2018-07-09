#! usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import time
import json
import sys
sys.path.append('../')
from audio.turtlebot_audio import TurtlebotAudio

class Move(object):
    def __init__(self):

        rospy.on_shutdown(self.shutdown)

        self.MODE_STILL = 0 # quieto
        self.MODE_MOVE1 = 1 # move absoluto
        self.MODE_MOVE2 = 2 # girar en posicion al final
        self.MODE_MOVE3 = 3 # move relativo (locate)

        self.robot_state = self.MODE_STILL

        self.cmd_vel = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=5)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/lista_goals", String, self.callback_goal)
        rospy.Subscriber("/lista_goals_locate", String, self.callback_goals_locate)
        rospy.Subscriber("/location", String, self.locationCallback)
        # rospy.Subscriber("/walls", String, self.callback_walls)

        self.goal_actual_publisher = rospy.Publisher("/goal_actual",String, queue_size=1)

        #variables del robot
        #move message para publicar
        self.move_cmd = Twist()
        #posicion actual x,y,z
        self.pos = [0,0,0]
        #posicion actual angular
        self.ang = 0

        self.last_vel_x = 0
        self.last_vel_theta = 0

        self.lista_comandos = []

        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        self.r = rospy.Rate(100)

        self.goals = []
        self.goals_locate = []
        self.obstacle = False
        self.obstacle_pos = None #"Left" o "Right"

        self.aux_time_obst = None

        self.comandos_anteriores_obst = ["False_None", "False_None"]

        self.evading = False

        self.avoiding_wall = False
        self.avoiding_wall_t0 = 0

        self.pose_mapa = None # pose del robot en el mapa, segun callback de localizacion

        self.soundplay = TurtlebotAudio()

        self.arrived = False

        # self.goals_cambiados = False


    def angle_from_robot(self,x,y):
        return math.atan2(y,x)


    def stop(self):
        self.cmd_vel.publish(Twist())

    def shutdown(self):
        self.stop()
        rospy.sleep(0.5)

    def odom_callback(self, data):
        pos = data.pose.pose.position
        self.pos = [pos.x , pos.y, pos.z]

        angaux = data.pose.pose.orientation.w
        self.ang = 2 * math.acos( angaux )
        if data.pose.pose.orientation.z < 0:
           self.ang = 2*math.pi - self.ang

        # self.pose_mapa = {'x':self.pos[0],'y':self.pos[1],'theta':self.ang}

        #print self.pos
        #print self.ang
        #print ""

    def callback_goal(self,data):
        # print('CALLBACK goals')
        if not self.evading:
            goals_list = json.loads(str(data.data))
            self.goals = []
            for goal in goals_list:
                goal_mapa = [goal['x'], goal['y'],goal["theta"],goal['isGoal']]
                self.goals.append(goal_mapa)
            self.mode = True
            self.robot_state = self.MODE_MOVE1
            self.goals_locate = []

    def callback_goals_locate(self,data):
        # print('CALLBACK goals locate')
        if not self.evading:
            goals_list = json.loads(str(data.data))
            self.goals_locate = []
            for goal in goals_list:
                pose_relativa = [goal['x'], goal['y'],goal["theta"]]
                self.goals_locate.append([pose_relativa,self.absolute_pos(pose_relativa)])
            self.mode = True
            self.robot_state = self.MODE_MOVE3
            self.goals = []


    def absolute_pos(self, pose):
        p_rot = self.rotate((0,0),pose,self.ang)
        theta_abs = pose[2]+self.ang
        if theta_abs > 2*math.pi:
            theta_abs = 2*math.pi - theta_abs
        return (p_rot[0]+self.pos[0],p_rot[1]+self.pos[1],theta_abs)

    def relative_pos_map(self,pose_absoluta_obj,pose_mapa):
        punto = (pose_absoluta_obj[0]-pose_mapa['x'],pose_absoluta_obj[1]-pose_mapa['y'])
        p_rot = self.rotate((0,0),punto,-pose_mapa['theta'])
        return p_rot
    
    def relative_pos_locate(self,pose_absoluta_obj):
        punto = (pose_absoluta_obj[0]-self.pos[0],pose_absoluta_obj[1]-self.pos[1])
        p_rot = self.rotate((0,0),punto,-self.ang)
        return p_rot

    def callback_obstacle_path_finding(self, data):
        if not self.avoiding_wall:
            self.avoiding_wall = True
            goal_obstacle = json.loads(str(data.data))[0]
            goal_obstacle = [goal_obstacle['x'], goal_obstacle['y'],goal_obstacle["theta"],goal_obstacle['isGoal']]
            self.goals.insert(0, goal_obstacle) 
            self.avoiding_wall_t0 = time.time()
            self.robot_state = self.MODE_MOVE1
            self.goals_locate = []

    # def callback_obstaculo(self, data):
    #     res = str(data.data)
    #     booleano = res.split("_")[0]

    #     self.comandos_anteriores_obst.append(res)
    #     self.comandos_anteriores_obst.pop(0)

    #     # filtro de suavidad
    #     if all(self.comandos_anteriores_obst[0] == val for val in self.comandos_anteriores_obst):
    #         res = self.comandos_anteriores_obst[0]
    #         booleano = res.split("_")[0]
    #     else:
    #         booleano = "False"

    #     if booleano == "True":
    #         # print(res)
    #         self.obstacle = False #True
    #         self.obstacle_pos = res.split("_")[1]

    #     else:
    #         time_turning = 0.6
    #         if len(self.goals) == 2 and self.aux_time_obst is not None and time.time() - self.aux_time_obst > time_turning:
    #             self.goals.pop(0)
    #             self.evading = False
    #         self.obstacle = False
    #         self.obstacle_pos = None

    # def evade_obstacle(self, obstacle_pos):
    #     cte_desvio_y = 2
    #     cte_desvio_x = 0.3
    #     self.evading = True
    #     if len(self.goals) == 1:  # primera vez que ve obstaculo.
    #         pose_relativa = [cte_desvio_x, cte_desvio_y, 0]
    #         if obstacle_pos == "Left":
    #             pose_relativa = [cte_desvio_x, -cte_desvio_y, 0]
    #         self.goals = [[pose_relativa, self.absolute_pos(pose_relativa)], [self.relative_pos_locate(self.goals[0][1]),self.goals[0][1]]]
    #     else:  # ya vio obstaculo anteriormente.
    #         pose_relativa = [cte_desvio_x, cte_desvio_y, 0]
    #         if obstacle_pos == "Left":
    #             pose_relativa = [cte_desvio_x, -cte_desvio_y, 0]
    #         self.goals = [[pose_relativa, self.absolute_pos(pose_relativa)], [self.relative_pos_locate(self.goals[1][1]),self.goals[1][1]]]


    def rotate(self,origin, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.

        The angle should be given in radians.
        """
        ox, oy = origin
        px, py = (point[0],point[1])

        qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
        qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
        return qx, qy

    def locationCallback(self, data):
        pos = json.loads(str(data.data))
        if pos is not None:
            self.pose_mapa = {'x': pos['x'], 'y': pos['y'], 'theta': pos['theta']}
        else:
            self.pose_mapa = None

    def calculate_error(self, pose_absoluta):
        pose_obj = self.relative_pos_map(pose_absoluta,self.pose_mapa)
        
        error_x = pose_obj[0]
        error_y = pose_obj[1]
        error_theta = self.angle_from_robot(error_x, error_y)


        # pose_real = self.absolute_pos(pose_relativa)
        # print(self.relative_pos(pose_absoluta))
        if error_theta > math.pi:
            error_theta = -(2*math.pi - error_theta)
        elif error_theta < -math.pi:
            error_theta = 2*math.pi + error_theta

        # print("ERROR_THETA: ",error_theta)

        return (error_x, error_y, error_theta )

    def calculate_error_locate(self, pose_relativa, pose_absoluta):
        pose_obj = self.relative_pos_locate(pose_absoluta)
        error_x = pose_obj[0]
        error_y = pose_obj[1]
        error_theta = self.angle_from_robot(error_x, error_y)

        if error_theta > math.pi:
            error_theta = -(2*math.pi - error_theta)
        elif error_theta < -math.pi:
            error_theta = 2*math.pi + error_theta

        return (error_x, error_y, error_theta )

    def controlled_tick(self, event):

        if self.avoiding_wall:
            t_elapsed = time.time() - self.avoiding_wall_t0
            if t_elapsed > 0.6:
                self.avoiding_wall_t0 = 0
                self.avoiding_wall = False
                self.goals.pop(0)

        # print(self.pos)
        # print(self.ang)
        # print("")
        # print(self.robot_state, len(self.goals))
        # print("goal actual: "+self.goals[0])
        # print('goals:',len(self.goals),' goals locate:',len(self.goals_locate), ' robot_state:', self.robot_state)
        
        if self.robot_state == self.MODE_STILL and len(self.goals) > 0:
            self.robot_state = self.MODE_MOVE1

        if self.pose_mapa is None:
            self.robot_state = self.MODE_STILL
            self.goals = []

        if self.robot_state == self.MODE_STILL and len(self.goals_locate) > 0:
            self.robot_state = self.MODE_MOVE3
        
        if self.robot_state == self.MODE_MOVE3 and len(self.goals_locate) == 0:
            self.robot_state = self.MODE_STILL

        if len(self.goals) == 0 and len(self.goals_locate) == 0:
            self.robot_state = self.MODE_STILL

        vel_x = 0
        vel_theta = 0

        # if self.goals_cambiados:
        #     self.goals_cambiados = False

        self.const_p_x = 0.4
        self.const_p_theta = 0.6


        self.max_vel_x = 0.1
        self.min_vel_x = 0.05

        self.max_ac_x = 0.3
        self.max_ac_theta = 0.5

        self.max_vel_theta = 0.6
        self.min_vel_theta = 0.05

        thres_error_x = 0.3
        thres_error_theta = 4.0/180.0*math.pi #10 grados.

        if len(self.goals) > 0:
            self.goal_actual_publisher.publish(String(json.dumps(self.goals[0])))
        elif len(self.goals_locate) > 0:
            self.goal_actual_publisher.publish(String(json.dumps(self.goals_locate[0])))

        if self.robot_state == self.MODE_MOVE1:
            # print("GOAL",self.goals[0])
            
            #error con pose absoluta.
            # try:
            error = self.calculate_error(self.goals[0]) #retorna tupla (error_x,error_y,error_theta)

            #Solo consideramos el error en x y el error de mi theta actual comparado
            #con mi angulo hacia el objetivo.
            vel_x = error[0]*self.const_p_x
            vel_theta = error[2]*self.const_p_theta

            # print(vel_x, vel_theta)

            # print("vel_x",vel_x)

            if abs(vel_x) > self.max_vel_x:
                if vel_x > 0:
                    vel_x = self.max_vel_x
                else:
                    vel_x = -self.max_vel_x
            # elif error[0] < thres_error_x:
            #     vel_x = 0
            elif abs(error[0]) > thres_error_x and abs(vel_x) < self.min_vel_x:
                if vel_x < 0:
                    vel_x = -self.min_vel_x
                else:
                    vel_x = self.min_vel_x
            if abs(vel_theta) > self.max_vel_theta:
                if vel_theta < 0:
                    vel_theta = -self.max_vel_theta
                else:
                    vel_theta = self.max_vel_theta
            elif abs(error[2]) > thres_error_theta and abs(vel_theta) < self.min_vel_theta:
                if vel_theta < 0:
                    vel_theta = -self.min_vel_theta
                else:
                    vel_theta = self.min_vel_theta

            # print(vel_x, vel_theta)

            # if error[0] < thres_error_x:
            #     vel_x = 0
            # if error[1] < thres_error_theta:
            #     vel_theta = 0

            if abs(error[0]) < thres_error_x and abs(error[1]) < thres_error_x:# and abs(error[2]) < thres_error_theta:

                # print("termine goal",self.goals[0])
                # self.pose_mapa['x'] += 0.2
                
                isGoal = self.goals[0][3]
                if not isGoal:
                    vel_theta = 0
                    self.goals.pop(0)
                    if self.avoiding_wall:
                        self.avoiding_wall = False
                        self.avoiding_wall_t0 = 0
                else:
                    if abs(error[0]) < thres_error_x*0.5 and abs(error[1]) < thres_error_x*0.5:
                        vel_theta = 0
                        self.goals.pop(0)
                        if self.avoiding_wall:
                            self.avoiding_wall = False
                            self.avoiding_wall_t0 = 0
                
                if len(self.goals) == 0:
                    if isGoal:
                        self.arrived = True
                        self.soundplay.say('Finished')
                        self.robot_state = self.MODE_STILL
                    
                    # self.robot_state = self.MODE_STILL
                # else:
                #     self.goals[0][1] = self.absolute_pos(self.goals[0][0])

        elif self.robot_state == self.MODE_MOVE3:
            #error con pose absoluta.
            # try:
            # self.const_p_theta = 0.7
            # thres_error_x = 0.16
            thres_error_theta = 10.0/180.0*math.pi 

            error = self.calculate_error_locate(self.goals_locate[0][0],self.goals_locate[0][1]) #retorna tupla (error_x,error_theta,error_y)
            
            # except:
            # self.robot_state = self.MODE_STILL
            # return
            # print(error)
            # print("")

            #Solo consideramos el error en x y el error de mi theta actual comparado
            #con mi angulo hacia el objetivo.
            vel_x = error[0]*self.const_p_x
            vel_theta = error[2]*self.const_p_theta

            if abs(vel_x) > self.max_vel_x:
                if vel_x > 0:
                    vel_x = self.max_vel_x
                else:
                    vel_x = -self.max_vel_x
            # elif error[0] < thres_error_x:
            #     vel_x = 0
            elif abs(error[0]) > thres_error_x and abs(vel_x) < self.min_vel_x:
                if vel_x < 0:
                    vel_x = -self.min_vel_x
                else:
                    vel_x = self.min_vel_x
            if abs(vel_theta) > self.max_vel_theta:
                if vel_theta < 0:
                    vel_theta = -self.max_vel_theta
                else:
                    vel_theta = self.max_vel_theta
            elif abs(error[2]) > thres_error_theta and abs(vel_theta) < self.min_vel_theta:
                if vel_theta < 0:
                    vel_theta = -self.min_vel_theta
                else:
                    vel_theta = self.min_vel_theta


            # if error[0] < thres_error_x:
            #     vel_x = 0
            # if error[1] < thres_error_theta:
            #     vel_theta = 0

            if abs(error[0]) < thres_error_x and abs(error[1]) < thres_error_x:
                vel_theta = 0
                # self.goals_list.pop(0)
                self.robot_state = self.MODE_STILL

            if error[0] < 0:
                print("error",error)
                print(vel_x,vel_theta)

        elif self.robot_state == self.MODE_MOVE2:
            ##ejecutar movimiento para llegar a pose theta
            error_theta = self.goals[0][2]-self.pose_mapa['theta']
            if error_theta > math.pi:
                error_theta = -(2*math.pi - error_theta)
            elif error_theta < -math.pi:
                error_theta = 2*math.pi + error_theta

            # print(abs(error_theta), thres_error_theta)
            # print("Error theta: ",error_theta)

            vel_x = 0
            vel_theta = error_theta*self.const_p_theta

            if abs(vel_theta) > self.max_vel_theta:
                if vel_theta < 0:
                    vel_theta = -self.max_vel_theta
                else:
                    vel_theta = self.max_vel_theta
            elif abs(error_theta) > thres_error_theta and abs(vel_theta) < self.min_vel_theta:
                if vel_theta < 0:
                    vel_theta = -self.min_vel_theta
                else:
                    vel_theta = self.min_vel_theta
            elif abs(error_theta) < thres_error_theta:
                vel_theta = 0
                self.goals.pop(0)
                # print("Llego a goal")
                self.robot_state = self.MODE_STILL
                
        if event.last_real != None:
            time_elapsed = time.time() - event.last_real.to_sec()

            ac_x = (vel_x - self.last_vel_x)*1.0/time_elapsed
            ac_theta = (vel_theta - self.last_vel_theta)*1.0/time_elapsed

            # print(ac_x,ac_theta)

            if abs(ac_x) > self.max_ac_x:
                ## velocidades en x son siempre mayor a 0
                # if vel_x > self.last_vel_x:
                if ac_x > 0:
                    vel_x = (self.max_ac_x*time_elapsed) + self.last_vel_x
                else:
                    vel_x = (-self.max_ac_x*time_elapsed) + self.last_vel_x

            if abs(ac_theta) > self.max_ac_theta:
                # if vel_theta < 0:
                if ac_theta > 0:
                    vel_theta = self.max_ac_theta*time_elapsed*1.0 + self.last_vel_theta
                else:
                    vel_theta = -self.max_ac_theta*time_elapsed*1.0 + self.last_vel_theta
                # else:
                    # vel_theta = -self.max_ac_theta*time_elapsed*1.0/(abs(vel_theta) - abs(self.last_vel_theta))


        # print(vel_theta)

        self.move_cmd.linear.x = vel_x
        self.move_cmd.angular.z = vel_theta

        self.last_vel_x = vel_x
        self.last_vel_theta = vel_theta

        self.cmd_vel.publish(self.move_cmd)
