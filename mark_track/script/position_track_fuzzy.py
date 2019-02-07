#!/usr/bin/env python

"""
Autor: David Valencia
Date : 07/01/2019


Description: Fuzzy Control with AR mark Tracking for Parrot Bebop Drone

"""

from geometry_msgs.msg        import Twist 
from ar_pose.msg              import ARMarker
from transformations_to_euler import *

import rospy
import time 
import numpy           as np
import skfuzzy         as fuzz
import skfuzzy.control as ctrl


class Lecture_Marker:

	def __init__(self):

		rospy.Subscriber("ar_pose_marker", ARMarker , self.mark_position)
		self.movePub = rospy.Publisher ("bebop/cmd_vel", Twist, queue_size=10)


		self.posx = 0.0
		self.posy = 0.0
		self.posz = 0.0
		self.yaw  = 0.0

		self.X_d   = 0.0
		self.Y_d   = -0.4
		self.Z_d   = 2.0
		self.yaw_d = 0.0

		self.flag_1 = False
		self.flag_2 = False 
		self.tie    = 0

		# fuzzy axis Z ##(ar_mark frame) 
		#--------------------------------------------------------------------------------------------------
		error_z_f = ctrl.Antecedent(np.arange(  -3,   3, 0.01), 'error_z')
		output_z  = ctrl.Consequent(np.arange(-0.3, 0.3, 0.01), 'output')


		error_z_f['neg_big']    = fuzz.zmf(error_z_f.universe     , -3.0, -1.4)
		error_z_f['neg_medio']  = fuzz.trimf(error_z_f.universe   ,[-2.2, -1.4 , -0.6])
		error_z_f['neg_small']  = fuzz.trimf(error_z_f.universe   ,[-1.0, -0.6 , -0.2])
		error_z_f['neg_bit']    = fuzz.trimf(error_z_f.universe   ,[-0.4, -0.2 ,  0.0])
		error_z_f['mantener_z'] = fuzz.trimf(error_z_f.universe   ,[-0.1, 0.0, 0.1])
		error_z_f['pos_bit']    = fuzz.trimf(error_z_f.universe   ,[0.0, 0.2, 0.4])
		error_z_f['pos_small']  = fuzz.trimf(error_z_f.universe   ,[0.2, 0.6, 1.0])
		error_z_f['pos_medio']  = fuzz.trimf(error_z_f.universe   ,[0.6, 1.4, 2.2])
		error_z_f['pos_big']    = fuzz.smf(error_z_f.universe     ,1.4 , 3.0 )

		output_z['mo_atras_big']      = fuzz.zmf(output_z.universe      ,-0.30 , -0.14)
		output_z['mo_atras_medio']    = fuzz.trimf(output_z.universe   ,[-0.22,  -0.14, -0.06]) 
		output_z['mo_atras_small']    = fuzz.trimf(output_z.universe   ,[-0.10,  -0.06,  -0.02])
		output_z['mo_atras_bit']      = fuzz.trimf(output_z.universe   ,[-0.04,  -0.02,  -0.00])
		output_z['mantener_z']        = fuzz.trimf(output_z.universe   ,[-0.01,   0.00,  0.01]) 
		output_z['mo_adelante_bit']   = fuzz.trimf(output_z.universe   ,[0.00,  0.02, 0.04]) 
		output_z['mo_adelante_small'] = fuzz.trimf(output_z.universe   ,[0.02,  0.06, 0.10]) 
		output_z['mo_adelante_medio'] = fuzz.trimf(output_z.universe   ,[0.06,  0.14, 0.22]) 
		output_z['mo_adelante_big']   = fuzz.smf(output_z.universe     , 0.14 , 0.30 )

		rule0 = ctrl.Rule(antecedent=((error_z_f['neg_big']))   , consequent=output_z['mo_adelante_big'])
		rule1 = ctrl.Rule(antecedent=((error_z_f['neg_medio'])) , consequent=output_z['mo_adelante_medio'])
		rule2 = ctrl.Rule(antecedent=((error_z_f['neg_small'])) , consequent=output_z['mo_adelante_small'])
		rule3 = ctrl.Rule(antecedent=((error_z_f['neg_bit']))   , consequent=output_z['mo_adelante_bit'])
		rule4 = ctrl.Rule(antecedent=((error_z_f['mantener_z'])), consequent=output_z['mantener_z'])
		rule5 = ctrl.Rule(antecedent=((error_z_f['pos_bit']))   , consequent=output_z['mo_atras_bit'])
		rule6 = ctrl.Rule(antecedent=((error_z_f['pos_small'])) , consequent=output_z['mo_atras_small'])
		rule7 = ctrl.Rule(antecedent=((error_z_f['pos_medio'])) , consequent=output_z['mo_atras_medio'])
		rule8 = ctrl.Rule(antecedent=((error_z_f['pos_big']))   , consequent=output_z['mo_atras_big'])

		self.system        = ctrl.ControlSystem(rules=[rule0, rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8])
		self.system_result = ctrl.ControlSystemSimulation(self.system)
		#--------------------------------------------------------------------------------------------------


		# fuzzy axis X ## (ar_mark frame)
		#--------------------------------------------------------------------------------------------------


		error_x_f = ctrl.Antecedent(np.arange( -1.5,  1.5, 0.01), 'error_x')
		output_x  = ctrl.Consequent(np.arange( -0.2,  0.2, 0.01), 'output_x') #

		error_x_f['er_big_izquie']   = fuzz.zmf(error_x_f.universe     , -1.5, -0.6)
		error_x_f['er_small_izquie'] = fuzz.trimf(error_x_f.universe   ,[-1.0, -0.6, -0.2])
		error_x_f['re_small_izquie'] = fuzz.trimf(error_x_f.universe   ,[-0.4, -0.2,  0.0])
		error_x_f['centro_x']        = fuzz.trimf(error_x_f.universe   ,[-0.1,  0.0, 0.1])
		error_x_f['re_small_derec']  = fuzz.trimf(error_x_f.universe  ,[0.0, 0.2, 0.4])
		error_x_f['er_small_derec']  = fuzz.trimf(error_x_f.universe  ,[0.2, 0.6, 1.0])
		error_x_f['er_big_derech']   = fuzz.smf(error_x_f.universe    , 0.6,  1.5 )

		output_x['mo_izq_big']   = fuzz.zmf(output_x.universe      ,-0.20 , -0.06)
		output_x['mo_izq_small'] = fuzz.trimf(output_x.universe   ,[-0.10,  -0.06, -0.02]) 
		output_x['mo_izq_bit']   = fuzz.trimf(output_x.universe   ,[-0.04,  -0.02,  0.00])
		output_x['mantener_x']   = fuzz.trimf(output_x.universe   ,[-0.01,   0.00,  0.01]) 
		output_x['mo_der_bit']   = fuzz.trimf(output_x.universe   ,[0.00,  0.02, 0.04]) 
		output_x['mo_der_small'] = fuzz.trimf(output_x.universe   ,[0.02,  0.06, 0.10])
		output_x['mo_der_big']   = fuzz.smf(output_x.universe     , 0.06 , 0.20 )

		rule1_x = ctrl.Rule(antecedent=((error_x_f['er_big_izquie']))   , consequent=output_x['mo_der_big'])
		rule2_x = ctrl.Rule(antecedent=((error_x_f['er_small_izquie'])) , consequent=output_x['mo_der_small'])
		rule3_x = ctrl.Rule(antecedent=((error_x_f['re_small_izquie'])) , consequent=output_x['mo_der_bit'])
		rule4_x = ctrl.Rule(antecedent=((error_x_f['centro_x']))        , consequent=output_x['mantener_x'])
		rule5_x = ctrl.Rule(antecedent=((error_x_f['re_small_derec']))  , consequent=output_x['mo_izq_bit'])
		rule6_x = ctrl.Rule(antecedent=((error_x_f['er_small_derec']))  , consequent=output_x['mo_izq_small'])
		rule7_x = ctrl.Rule(antecedent=((error_x_f['er_big_derech']))   , consequent=output_x['mo_izq_big'])

		self.system_x        = ctrl.ControlSystem(rules=[rule1_x, rule2_x, rule3_x, rule4_x, rule5_x, rule6_x, rule7_x])
		self.system_result_x = ctrl.ControlSystemSimulation(self.system_x)
		
		#--------------------------------------------------------------------------------------------------



		# fuzzy axis Y ## (ar_mark frame) 
		#--------------------------------------------------------------------------------------------------
		error_y_f  = ctrl.Antecedent(np.arange(-4  , 4  , 0.01), 'error_y')
		output_y   = ctrl.Consequent(np.arange(-0.3, 0.3, 0.01), 'output_y')
		

		error_y_f['little_down']  = fuzz.zmf(error_y_f.universe     ,-2.5  ,  -0.05 )
		error_y_f['center_y']     = fuzz.trimf(error_y_f.universe   ,[-0.1, 0.0, 0.1])
		error_y_f['little_hight'] = fuzz.smf(error_y_f.universe     ,0.05 ,  2.5 )

		output_y['up_small']   = fuzz.zmf(output_y.universe      ,-0.3 , 0.0 )
		output_y['holder_y']   = fuzz.gaussmf(output_y.universe  ,  0  , 0.02)  
		output_y['down_small'] = fuzz.smf(output_y.universe      ,0.0  , 0.3 )
  
		rule0_y = ctrl.Rule(antecedent=((error_y_f['little_hight'])), consequent=output_y['down_small'])
		rule1_y = ctrl.Rule(antecedent=((error_y_f['center_y']))    , consequent=output_y['holder_y'])
		rule2_y = ctrl.Rule(antecedent=((error_y_f['little_down'])) , consequent=output_y['up_small'])
		
		self.system_y        = ctrl.ControlSystem(rules=[rule0_y, rule1_y, rule2_y])
		self.system_result_y = ctrl.ControlSystemSimulation(self.system_y)

		#--------------------------------------------------------------------------------------------------



	def mark_position(self, data) :

		self.posx = data.pose.pose.position.x
		self.posy = data.pose.pose.position.y
		self.posz = data.pose.pose.position.z

		euler    = get_yaw_from_quaternion(data.pose.pose.orientation) 
		self.yaw = euler[1] / numpy.pi * 180.0

		self.tie = data.header.seq

		if data.id == 0: # this if is to verify that it read the correct mark
						 # if a wrong reading is given it does not affect

			self.flag_1 = True
		else:
			self.flag_1 = False

	def calculos(self):

		rate    = rospy.Rate(10) # 10hz 
		actual  = 0
		last    = 0

		PD_Yaw  = pd_controller(0.002,  0.00, 0.9)


		while not rospy.is_shutdown():
			
			error_x   = self.X_d    - self.posx
			error_y   = self.Y_d    - self.posy 
			error_z   = self.Z_d    - self.posz
			error_yaw = self.yaw_d  - self.yaw
 			
			if   error_z >=  3 :
				 error_z  =  3

			elif error_z <= -3 :
				 error_z  = -3

			if   error_x >=  1.5 :
				 error_x  =  1.5

			elif error_x <= -1.5 :
				 error_x  = -1.5

			if   error_y >=  4 :
				 error_y  =  4

			elif error_y <= -4 :
				 error_y  = -4

			
				 
			U2 = self.set_fuzzy_z(error_z) 
			U1 = self.set_fuzzy_y(error_y)
			U4 = PD_Yaw.set_current_error(error_yaw)
			U3 = self.set_fuzzy_x(error_x) 

			actual = self.tie

			if actual == last:# este if es para evitar que se quede la ultima lectura
							  # en el caso de perder de vista a la marca
		
				self.flag_2 = False
			else:
				self.flag_2 = True

			last   =  actual 	

			self.Action(U1,U2,U3,U4)
			rate.sleep()


	def Action (self, U1, U2, U3, U4):

		twist = Twist()
		
		if (self.flag_1 == True and self.flag_2 == True) :

			twist.linear.z  =  U1
			twist.linear.x  =  U2
			twist.linear.y  =  -U3
			twist.angular.z =  -U4
		else :
			twist.linear.z  = 0
			twist.linear.x  = 0
			twist.linear.y  = 0
			twist.angular.z = 0
		
		self.movePub.publish(twist)

   
	def set_fuzzy_z(self, er):
		
		self.system_result.input['error_z'] = er
		self.system_result.compute()

		resultado_fuzzy = self.system_result.output
		a = str(resultado_fuzzy)
		b = a [24:-3]
		resultado = float (b)

		
		return resultado

	def set_fuzzy_x(self, er):
		
		self.system_result_x.input['error_x'] = er
		self.system_result_x.compute()

		resultado_fuzzy = self.system_result_x.output

		a = str(resultado_fuzzy)
		b = a [25:-3]
		resultado = float (b)

		
		return resultado

	def set_fuzzy_y(self, er):
		
		self.system_result_y.input['error_y'] = er
		self.system_result_y.compute()

		resultado_fuzzy = self.system_result_y.output

		a = str(resultado_fuzzy)
		b = a [25:-3]
		resultado = float (b)
		
		return resultado

class pd_controller:

	def __init__(self, p_coef, d_coef, limit_out):
		
		self.kp = p_coef
		self.kd = d_coef

		self._limit_out 	 = limit_out
		self._previous_error = 0.0
		self._is_error_initialized = False


	def set_current_error(self, error):
	
		output = error * self.kp


		if self._is_error_initialized:

			error_diff = error - self._previous_error
			output += self.kd * error_diff
			self._previous_error = error


		else:
			
			self._previous_error = error
			self._is_error_initialized = True

		if output > self._limit_out:
			output = self._limit_out
		elif output < (-self._limit_out):
			output = (-self._limit_out)


		return output

def shutdown_callback():
	print "Shutting down position controller."




if __name__ == "__main__":

	rospy.init_node('controller_ar_pose_marker',anonymous=True)
	a = Lecture_Marker()
	a.calculos()
	rospy.on_shutdown(shutdown_callback)
	rospy.spin()
	

