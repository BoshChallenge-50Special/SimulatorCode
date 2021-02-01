#!/usr/bin/python

"""
These are the libraries you need to import in your project in order to
be able to communicate with the Gazebo simulator
"""

from LineTracking import ParticleFilter

class ControlUnit(object):

	def __init__(self, cam, car, sem, gps, bno, parameters={}):
		self.cam=cam
		self.car=car
		self.sem=sem
		self.gps=gps
		self.bno=bno

	def start():
		#Read input from all modules
		line_tracking(True)
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
