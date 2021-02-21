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

			#THREAD: 2. Duplicate the following line. Change the pipeline name and add or remove the other parameters
			#			depending on the parameter of your Function
			#			self.line_tracking is a method that configure the module and start it
			executor.submit(self.line_tracking, pipeline_lines, event, False)
			#time.sleep(0.1)
			#logging.info("Main: about to set event")
			#event.set()

			while(True):
				#THREAD 3. Read the data received from the thread from your pipeline
				#THREAD 4. In your module use  to load data in the queue: data_queue.put(...stuff...)
				lines=pipeline_lines.get()
				#print(lines[0].toString()+"   "+lines[1].toString())  #CHECK DATA
			#Read input from all modules
			#line_tracking(True)
			#get_location()
			#Do some control action

	def line_tracking(self, data_queue, event, verbose=False):
		N_particles           = 50  #100 # Particles used in the filter
		Interpolation_points  = 17  #25  # Interpolation points used for the spline
		order                 = 2        # Spline order
		N_c                   = 3        # Number of spline control points

		ParticleFilter.filter_usage_BOSH(N_Particles=N_particles,
							Interpolation_points=Interpolation_points,
							order=1,
							N_points=2,
							Images_print=verbose,
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
