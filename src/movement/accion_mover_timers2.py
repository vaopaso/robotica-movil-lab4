#! usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import time
import json
import sys

class Move(object):
    def __init__(self):

        rospy.on_shutdown(self.shutdown)

        self.MODE_STILL = 0
        self.MODE_MOVE1 = 1
        self.MODE_MOVE2 = 2

        self.robot_state = self.MODE_STILL

        self.cmd_vel = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=5)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/lista_goals", String, self.callback_goal)
        # rospy.Subscriber("/walls", String, self.callback_walls)


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
        self.obstacle = False
        self.obstacle_pos = None #"Left" o "Right"

        self.aux_time_obst = None

        self.comandos_anteriores_obst = ["False_None", "False_None"]

        self.evading = False

        # self.goals_cambiados = False

    # def callback_walls(self, data):
    #     values = json.loads(str(data.data))
    #     minval_left = values['minval_left']
    #     minval_right = values['minval_right']
    #
    #     print(values) # en centimetros
    #
    #     threshold = 600 # solo se publica un goal en caso que algun lado este a menos del threshold
    #
    #     print(minval_left < threshold or minval_right < threshold)
    #
    #
    #     if minval_left < threshold or minval_right < threshold:
    #         diff = minval_left - minval_right
    #         if diff > 0: # lado izq mayor (mas lejos). Moverse a la izq.
    #             dir = 1 # dir se multiplicara a la magnitud de la velocidad angular
    #         else:
    #             dir = -1
    #
    #         x = 25*(math.exp(0.009*diff)-1)
    #         y = dir*(-25)*(math.exp(0.0085*diff)-1)+2000
    #
    #         print('{0}, {1}, {2}'.format(diff, x, y))
    #
    #         pose_relativa = [x*1.0/1000, y*1.0/1000, 0]
    #         # self.goals.append([pose_relativa, self.absolute_pos(pose_relativa)])


    def angle_from_robot(self,x,y):
        return math.atan2(y,x)


    # def move_robot_goal(self, goal_pose):
    #     y = goal_pose[0]
    #     x = goal_pose[1]
    #     theta = goal_pose[2]
    #     theta_1 = self.angle_from_robot(x,y)
    #
    #
    #     if abs(theta_1 ) < 0.25 and rospy.get_param('is_object_detection_alive'):
    #         # print(rospy.get_param('is_object_detection_alive'))
    #         theta_1 = 0
    #
    #     theta_2 = theta - theta_1
    #
    #     angle_factor = 0.35
    #     #factor de correccion de angulo
    #     if theta_1 < 0:
    #         theta_1 -= angle_factor
    #     elif theta_1 > 0:
    #         theta_1 += angle_factor
    #
    #
    #
    #     angular_vel1 = 1
    #     angular_vel2 = 1
    #     linear_vel = 0.2
    #     angular_t1 = theta_1/angular_vel1
    #     dist = math.sqrt(x*x + y*y)
    #     linear_t = dist/linear_vel
    #
    #
    #     angular_t2 = theta_2/angular_vel2
    #
    #     if angular_t1 < 0:
    #         angular_t1 = -angular_t1
    #         angular_vel1 = -angular_vel1
    #
    #     if angular_t2 < 0:
    #         angular_t2 = -angular_t2
    #         angular_vel2 = -angular_vel2
    #
    #     linear_factor = -0.03 #velocidad angular de correcion al ir derecho
    #
    #     return [[0,angular_vel1,angular_t1],[linear_vel,linear_factor,linear_t],[0,angular_vel2,angular_t2]]


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

        #print self.pos
        #print self.ang
        #print ""

    def callback_goal(self,data):
        if not self.evading:
            goals_list = json.loads(str(data.data))
            # print("nuevas goals: ",goals_list)
            # self.lista_comandos = []
            self.goals = []
            for goal in goals_list:
                pose_relativa = [goal['x'], goal['y'],goal["theta"]]
                self.goals.append([pose_relativa,self.absolute_pos(pose_relativa)])
                # self.lista_comandos += self.move_robot_goal(pose)
            # self.goals = goals_list
            self.mode = True
            self.robot_state = self.MODE_MOVE1
            # self.goals_cambiados = True

            # print("callback :)")
            # print(self.lista_comandos)

    def absolute_pos(self, pose):
        p_rot = self.rotate((0,0),pose,self.ang)
        theta_abs = pose[2]+self.ang
        if theta_abs > 2*math.pi:
            theta_abs = 2*math.pi - theta_abs
        return (p_rot[0]+self.pos[0],p_rot[1]+self.pos[1],theta_abs)

    def relative_pos(self,pose_absoluta_obj):
        punto = (pose_absoluta_obj[0]-self.pos[0],pose_absoluta_obj[1]-self.pos[1])
        p_rot = self.rotate((0,0),punto,-self.ang)
        return p_rot

    def callback_obstaculo(self, data):
        res = str(data.data)
        booleano = res.split("_")[0]

        self.comandos_anteriores_obst.append(res)
        self.comandos_anteriores_obst.pop(0)

        if all(self.comandos_anteriores_obst[0] == val for val in self.comandos_anteriores_obst):
            res = self.comandos_anteriores_obst[0]
            booleano = res.split("_")[0]
        else:
            booleano = "False"

        if booleano == "True":
            # print(res)
            self.obstacle = False #True
            self.obstacle_pos = res.split("_")[1]

            # ################ PARTE EVASION #############################
            # self.aux_time_obst = time.time()
            # self.evade_obstacle(self.obstacle_pos)
            # ############## FIN EVASION ################################

            # print(self.goals)
        else:
            time_turning = 0.6
            if len(self.goals) == 2 and self.aux_time_obst is not None and time.time() - self.aux_time_obst > time_turning:
                self.goals.pop(0)
                self.evading = False
            self.obstacle = False
            self.obstacle_pos = None

    def evade_obstacle(self, obstacle_pos):
        cte_desvio_y = 2
        cte_desvio_x = 0.3
        self.evading = True
        if len(self.goals) == 1:  # primera vez que ve obstaculo.
            pose_relativa = [cte_desvio_x, cte_desvio_y, 0]
            if obstacle_pos == "Left":
                pose_relativa = [cte_desvio_x, -cte_desvio_y, 0]
            self.goals = [[pose_relativa, self.absolute_pos(pose_relativa)], [self.relative_pos(self.goals[0][1]),self.goals[0][1]]]
        else:  # ya vio obstaculo anteriormente.
            pose_relativa = [cte_desvio_x, cte_desvio_y, 0]
            if obstacle_pos == "Left":
                pose_relativa = [cte_desvio_x, -cte_desvio_y, 0]
            self.goals = [[pose_relativa, self.absolute_pos(pose_relativa)], [self.relative_pos(self.goals[1][1]),self.goals[1][1]]]


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

    def calculate_error(self, pose_relativa, pose_absoluta):
        # print(pose_real)
        pose_obj = self.relative_pos(pose_absoluta)
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

    def controlled_tick(self, event):

        # print(self.pos)
        # print(self.ang)
        # print("")
        # print(self.robot_state, len(self.goals))
        # print("goal actual: "+self.goals[0])
        if self.robot_state == self.MODE_STILL and len(self.goals) > 0:
            self.robot_state = self.MODE_MOVE1

        if len(self.goals) == 0:
            self.robot_state = self.MODE_STILL

        vel_x = 0
        vel_theta = 0

        # if self.goals_cambiados:
        #     self.goals_cambiados = False

        self.const_p_x = 0.4
        self.const_p_theta = 1


        self.max_vel_x = 0.1
        self.min_vel_x = 0.05

        self.max_ac_x = 0.5
        self.max_ac_theta = 1

        self.max_vel_theta = 1
        self.min_vel_theta = 0.05

        thres_error_x = 0.02
        thres_error_theta = 3.0/180.0*math.pi #5 grados.

        if self.robot_state == self.MODE_MOVE1:
            #error con pose absoluta.
            # try:
            error = self.calculate_error(self.goals[0][0],self.goals[0][1]) #retorna tupla (error_x,error_theta,error_y)
            # except:
            # self.robot_state = self.MODE_STILL
            # return
            # print(error)
            # print("")

            #Solo consideramos el error en x y el error de mi theta actual comparado
            #con mi angulo hacia el objetivo.
            vel_x = error[0]*self.const_p_x
            vel_theta = error[2]*self.const_p_theta

            if vel_x > self.max_vel_x:
                vel_x = self.max_vel_x
            # elif error[0] < thres_error_x:
            #     vel_x = 0
            elif abs(error[0]) > thres_error_x and vel_x < self.min_vel_x:
                if vel_x < 0:
                    vel_x = -self.min_vel_x
                else:
                    vel_x = self.min_vel_x
            if abs(vel_theta) > self.max_vel_theta:
                if vel_theta < 0:
                    vel_theta = -self.max_vel_theta
                else:
                    vel_theta = self.max_vel_theta
            elif abs(error[1]) > thres_error_theta and abs(vel_theta) < self.min_vel_theta:
                if vel_theta < 0:
                    vel_theta = -self.min_vel_theta
                else:
                    vel_theta = self.min_vel_theta

            # if error[0] < thres_error_x:
            #     vel_x = 0
            # if error[1] < thres_error_theta:
            #     vel_theta = 0

            if abs(error[0]) < thres_error_x and abs(error[2]) < thres_error_theta:
                vel_theta = 0
                # self.goals_list.pop(0)
                self.robot_state = self.MODE_MOVE2

        elif self.robot_state == self.MODE_MOVE2:
            ##ejecutar movimiento para llegar a pose theta
            # print(self.goals)
            error_theta = self.goals[0][1][2]-self.ang
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
                if len(self.goals) > 0:
                    self.goals[0][1] = self.absolute_pos(self.goals[0][0])
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
