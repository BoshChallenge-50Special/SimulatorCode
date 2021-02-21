#!/usr/bin/python

"""
These are the libraries you need to import in your project in order to
be able to communicate with the Gazebo simulator
"""

from LineTracking import ParticleFilter

from Localization import GraphMap

class ControlUnit(object):

	def __init__(self, cam, car, sem, gps, bno, map_path, parameters={}):
		self.cam=cam
		self.car=car
		self.sem=sem
		self.gps=gps
		self.bno=bno
		self.map_path = map_path

	def start():
		#Read input from all modules
		line_tracking(True)
		get_location()
		#Do some control action

	def line_tracking(verbose=False):
		N_particles           = 50  #100 # Particles used in the filter
		Interpolation_points  = 17  #25  # Interpolation points used for the spline
		order                 = 2        # Spline order
		N_c                   = 3        # Number of spline control points

		ParticleFilter.filter_usage_BOSH(N_Particles=N_particles,
							Interpolation_points=Interpolation_points,
							order=1,
							N_points=2,
							Images_print=verbose,
							get_image_function=self.cam.getImage)


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

