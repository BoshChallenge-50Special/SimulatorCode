#!/usr/bin/python

"""
These are the libraries you need to import in your project in order to
be able to communicate with the Gazebo simulator
"""
import concurrent.futures
import Queue
import threading

from LineTracking import ParticleFilter

from Localization import GraphMap
from LineTracking.utils import Utils
import PID
import time

class ControlUnit(object):

	def __init__(self, cam, car, sem, gps, bno, map_path=None, parameters={}):
		self.cam=cam
		self.car=car
		self.sem=sem
		self.gps=gps
		self.bno=bno
		self.map_path = map_path

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
		pipeline_lines = Queue.Queue(maxsize=10)
		event = threading.Event()

		with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
			utils = Utils()

			#THREAD: 2. Duplicate the following line. Change the pipeline name and add or remove the other parameters
			#			depending on the parameter of your Function
			#			self.line_tracking is a method that configure the module and start it
			executor.submit(self.line_tracking, pipeline_lines, event, False)
			#time.sleep(0.1)
			#logging.info("Main: about to set event")
			#event.set()
			P = 0.1
			I = 0.002
			D = 0.002
			pid = PID.PID(P, I, D)
			while(True):
				#THREAD 3. Read the data received from the thread from your pipeline
				#THREAD 4. In your module use  to load data in the queue: data_queue.put(...stuff...)
				img=self.cam.getImage()
				lines=pipeline_lines.get()
				central_line=[(lines[0].spline[i][0]+img.shape[1]/2+lines[1].spline[i][0])/2 for i in range(0, len(lines[0].spline))]
				error_lane = central_line[3] - img.shape[1]/2
				steer=0
				#print(lines[0].spline)

				targetT = (img.shape[1]/2)#/img.shape[1]



				pid.SetPoint = targetT
				pid.setSampleTime(0.1)

				pid.update(central_line[17])#/img.shape[1])
				targetPwm = pid.output
				#targetPwm = max(min( int(targetPwm), 100 ),0)

				time.sleep(0.1)

				self.car.drive(0.17, -targetPwm)

				offsetApproximation=[]#[0,int(img.shape[1]/2 )]

				res_1 = utils.draw_particles(img, [], "Result", lines, start_coo=[[0, 300], [320, 300]])#, offset=[], offsetApproximation=offset_Approximation)

				#self.car.drive(0, targetPwm)
				#print("targetPwm : "+str(targetPwm)+ " error : "+str(error_lane))
				steer_dir=""
				if(-targetPwm > 0):
					steer_dir="right"
				else:
					steer_dir="left"
				print("COMMAND : VELOCITY = 0.17 | STEER = "+steer_dir+ " --> " +str(targetPwm))
				#print(lines[0].toString()+"   "+lines[1].toString())  #CHECK DATA
			#Read input from all modules
			#line_tracking(True)
			#get_location()
			#Do some control action

	def line_tracking(self, data_queue, event, verbose=False):
		N_particles           = 60  #100 # Particles used in the filter
		Interpolation_points  = 20  #25  # Interpolation points used for the spline
		order                 = 1        # Spline order
		N_c                   = 4       # Number of spline control points

		ParticleFilter.filter_usage_BOSH(N_Particles=N_particles,
							Interpolation_points=Interpolation_points,
							order=order,
							N_points=N_c,
							Images_print=verbose,
							threshold_reset=10,
							get_image_function=self.cam.getImage,
							data_queue=data_queue)



	# Algoritmo sulla linea obiettivo -> media all'inizio e poi aumentiamo la complessità con il path planning
	# Creare la linea attuale h/2 e trasformazione in IPM view.

	# Scrivere il codice per il PID sulla linea obiettivo (input = linea obiettivo + linea attuale)	|
	#																								| ---> Funzione controllo che ha in input il risultato del machine learning e il line tracking
	# Controllo stop e riduzione velocità della macchina											|

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
