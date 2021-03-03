#!/usr/bin/python

"""
These are the libraries you need to import in your project in order to
be able to communicate with the Gazebo simulator
"""
import concurrent.futures
import Queue
import threading
import traceback

from LineTracking import ParticleFilter
from TrafficSignDetection import traffic
from LineTracking.horizontal_line import HorizontalLine

from Localization import GraphMap
from LineTracking.utils import Utils
import PID
import time


import sys
sys.settrace

class ControlUnit(object):

	def __init__(self, cam, car, sem, gps, bno, map_path=None, parameters={}):
		self.cam=cam
		self.car=car
		self.sem=sem
		self.gps=gps
		self.bno=bno
		self.map_path = map_path
		self.pipeline_lines = Queue.Queue(maxsize=10)
		self.pipeline_signs = Queue.Queue(maxsize=10)
		self.pipeline_horizontal_line = Queue.Queue(maxsize=10)

	#def producer(queue, event):
    #"""Pretend we're getting a number from the network."""
    #while not event.is_set():
    #    message = random.randint(1, 101)
    #    logging.info("Producer got message: %s", message)
    #    queue.put(message)

    #logging.info("Producer received event. Exiting")

	#def consumer(queue, event):
	#    """Pretend we're saving a number in the database."""
	#    while #not event.is_set() or not queue.empty():
	#        message = queue.get()
	#        logging.info(
	#            "Consumer storing message: %s (size=%d)", message, queue.qsize()
	#        )
	#
    #	logging.info("Consumer received event. Exiting")

	def start(self):
		#THREAD: 1. Add a pipeline for your module

		event = threading.Event()

		with concurrent.futures.ThreadPoolExecutor(max_workers=10) as executor:
			utils = Utils()

			#THREAD: 2. Duplicate the following line. Change the pipeline name and add or remove the other parameters
			#			depending on the parameter of your Function
			#			self.line_tracking is a method that configure the module and start it
			executor.submit(self.line_tracking, self.pipeline_lines, event, True)
			executor.submit(self.sign_detection, self.pipeline_signs, event, True)
			executor.submit(self.horizontal_detection, self.pipeline_horizontal_line, event, True)
			executor.submit(self.execute_pid, event, True)
			#time.sleep(0.1)
			#logging.info("Main: about to set event")
			#event.set()
			P = 0.08
			I = 0.002
			D = 0.002
			pid = PID.PID(P, I, D)
			img=self.cam.getImage()
			steer=0
			targetT = 0+20 #(img.shape[1]/2)#/img.shape[1]

			try:
				while(True):
					#THREAD 3. Read the data received from the thread from your pipeline
					#THREAD 4. In your module use  to load data in the queue: data_queue.put(...stuff...)
					img = self.cam.getImage()

					lines = self.pipeline_lines.get()
					#signs = pipeline_signs.get()
					#print("size " + str(pipeline_lines.qsize()))

					distance_from_base = 0.6 #it's a percentage

					actual_trajectory=self.get_current_trajectory_point(lines, distance_from_base, size=img.shape[1])
					if(actual_trajectory==None):
						actual_trajectory=targetT
					steer=self.get_steer(pid, actual_trajectory, targetT)
					self.car.drive(0.18, -steer)
					time.sleep(0.1)

					offsetApproximation=[]#[0,int(img.shape[1]/2 )]
					res_1 = utils.draw_particles(img, [], "Result", lines, start_coo=[[0, 300], [320, 300]])#, offset=[], offsetApproximation=offset_Approximation))
					#error_lane = central_line[3] - img.shape[1]/2
					#print("targetPwm : "+str(targetPwm)+ " error : "+str(error_lane))
					steer_dir=""
					if(-steer > 0):
						steer_dir="right"
					else:
						steer_dir="left"
					#print("COMMAND : VELOCITY = 0.17 | STEER = "+steer_dir+ " --> " +str(steer) + " ERROR : "+str(targetT-actual_trajectory))
			#Read input from all modules
			#line_tracking(True)
			#get_location()
			#Do some control action
			except Exception as e:
				print(e)
				print(traceback.print_exc())

	def line_tracking(self, data_queue, event, verbose=True):
		N_particles           = 60  #100 # Particles used in the filter
		Interpolation_points  = 20  #25  # Interpolation points used for the spline
		order                 = 2        # Spline order
		N_c                   = 3       # Number of spline control points

		ParticleFilter.filter_usage_BOSH(N_Particles=N_particles,
							Interpolation_points=Interpolation_points,
							order=order,
							N_points=N_c,
							Images_print=verbose,
							threshold_reset=5,
							get_image_function=self.cam.getImage,
							data_queue=data_queue)

	def sign_detection(self, data_queue, event, verbose=True):
		try :
			sg = traffic.SignDetector(None,None, data_queue, self.cam.getImage)
			sg.run()
		except Exception as e:
			print(e)
			print(traceback.print_exc())
		#SignDetection.start_sign_detection(
		#					data_queue=data_queue)

	# Algoritmo sulla linea obiettivo -> media all'inizio e poi aumentiamo la complessit con il path planning
	#																								| ---> Funzione controllo che ha in input il risultato del machine learning e il line tracking
	# Controllo stop e riduzione velocit della macchina											|

	def get_location():
		Graph = GraphMap(self.map_path)

		gps_data = self.gps.getGpsData()

		#x = gps_data["coords"][0].real
		x = 1
		#y = gps_data["coords"][0].imag
		y = 2
		num = 5
		options = Graph.get_location_points(x, y, num)

		print(options)

	def get_current_trajectory_point_old(self, lines, distance_from_base, central_point=320):
		if(lines[0] != None and lines[1] != None):
			central_line=[(lines[0].spline[i][0]+central_point+lines[1].spline[i][0])/2 for i in range(0, len(lines[0].spline))]
			index = int(distance_from_base * len(central_line))
			return central_line[index]
		else:
			return central_point

	def get_current_trajectory_point(self, lines, distance_from_base, size=640):
		dist = []
		if(lines[0] != None):
			dist.append([lines[0].spline[i][0] for i in range(0, len(lines[0].spline))])
		if(lines[1] != None):
			dist.append([lines[1].spline[i][0]-size/2 for i in range(0, len(lines[1].spline))])

		if(len(dist)==0):
			return None
		elif(len(dist)==1):
			index = int(distance_from_base * len(dist[0]))
			return dist[0][index]
		else:
			too_near=False
			dist_avg = [(dist[0][i] + dist[1][i])/2 for i in range(0, len(dist[0])) ]
			index = int(distance_from_base * len(dist[0]))
			return dist_avg[index]

	def get_steer(self, pid, point_objective, targetT):
		pid.SetPoint = targetT
		pid.setSampleTime(0.1)
		pid.update(point_objective)#central_line[17])#/img.shape[1])
		targetPwm = pid.output

		return targetPwm

	def horizontal_detection(self, data_queue, event, verbose=True):
		try :
			horizonLine = HorizontalLine()
			while(True):
				img = self.cam.getImage().copy()
				horizonLine.check_horizontal(img, data_queue)
		except Exception as e:
			print(e)
			print(traceback.print_exc())

	def execute_pid(self, event, verbose=True):
		return
		#while(True):
			#print(self.pipeline_lines.get())
		#try :
		#	horizonLine = HorizontalLine()
		#	while(True):
		#		img = self.cam.getImage().copy()
		#		horizonLine.check_horizontal(img, data_queue)
		#except Exception as e:
		#	print(e)
		#	print(traceback.print_exc())
