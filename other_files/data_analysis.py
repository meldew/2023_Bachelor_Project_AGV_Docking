# The code is based on a generic approach for generating plots with circles and random points around them in Python.
# Adapted and implemented by Mikael Walde based on knowledge and examples from various programming resources.
# All method descriptions are made with AI "Mintlify Doc Writer for Python" extantion for VS Code.

import random
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# The `Presition` class generates a list of points based on a starting point and a list of distances,
# and plots the points with a circle representing the precision tolerance.
class Presition: 
	def __init__(self, center_point:list, distances:list, precition_tolerance:int, accuracy_distances:list):
		self.center_point = center_point
		self.accuracy_distances = accuracy_distances
		self.distances = distances
		self.precition_tolerance = precition_tolerance
		self.start_points = self.generate_points(self.center_point, self.accuracy_distances)
	
	def generate_points(self, start_points, distances) -> list:
		"""
		This function generates a list of points based on a starting point and a list of distances, with
		each point randomly placed at a certain distance and angle from the starting point.

		:param start_point: The starting point from which the distances will be measured and new points will
		be generated. It is a tuple containing the x and y coordinates of the starting point
		:param distances: A list of distances from the starting point to generate new points
		:return: The function `generate_points` returns a list of points. Each point is represented as a
		tuple of two values, the x-coordinate and y-coordinate. The number of points in the list is
		determined by the length of the `distances` parameter.
		"""
		points = []
		for distance in distances:
			angle = random.uniform(0, 2*math.pi)
			x = start_points[0] + distance * math.cos(angle)
			y = start_points[1] + distance * math.sin(angle)
			points.append((x, y))
		return points
	
	def Calculate_RSD(self, distances, precition_tolerance) -> list:
		"""
		The function calculates the relative standard deviation (RSD) of a set of distances based on a
		given precision tolerance.
		
		:param distances: a list of lists containing distance values between data points
		:param precition_tolerance: The precision tolerance is a value that represents the maximum
		allowable difference between the actual distance and the expected distance. If the absolute
		difference between the actual distance and the expected distance is less than or equal to the
		precision tolerance, then the distance is considered to be within the precision tolerance
		:return: a list of precision values calculated based on the input distances and precision
		tolerance.
		"""
		precision_list = []
		for row in distances:
			within_tolerance = sum([1 for distance in row if abs(distance) <= precition_tolerance])
			data_lenght = len(row)
			precision = within_tolerance / data_lenght * 100
			precision_list.append(precision)
		return precision_list

	def plot(self):
		"""
		This function generates a plot to simulate the precision of a robot from different start positions.
		"""
		random_points = [self.generate_points(self.start_points[i], self.distances[i]) for i in range(len(self.start_points))]
		figur, axes = plt.subplots()
		for i in range(len(self.start_points)):
			axes.add_patch(Circle(self.start_points[i], precition_tolerance, fill=True, color=['red', 'blue', 'green'][i], alpha=0.2))
			axes.plot(*zip(*random_points[i]), marker='o', color=['purple', 'blue', 'green'][i], linestyle='None')
			axes.plot(*zip(self.start_points[i]), marker='o', color=['red', 'red', 'red'][i], linestyle='None',linewidth=2)
		
		axes.set_aspect('equal', adjustable='box')
		plt.grid(color='black', linestyle=':', linewidth=0.5, alpha=0.5)
		plt.title('Simulating precition and accuracy of the robot from {} different start positions\n'
					'Where presition tolerance is set to {} mm'.format(len(self.start_points), precition_tolerance))
		axes.plot(*zip(self.center_point), marker='o', color='black', linestyle='None')
		plt.xlabel('mm\n'+ str(self.Calculate_RSD(distances,precition_tolerance))+ '% of cases, the values will fall within the {} mm tolerance range.'.format(precition_tolerance))
		plt.ylabel('mm')
		plt.show()
			
# example of use
distances = [[1,1,1,2,5,2,2,1,2,2,3,1,1,2,4,4,1,2,1,1], # First list is distances from first startpoint
             [2,5,3,4,2,2,3,2,4,2,3,1,3,1,1,1,1,3,1],   # Second list is distances from second startpoint
             [2,2,1,2,1,1,3,5,3,4,3,2,3,2,5,2,2,2,2,3]] # Third list is distances from third startpoint
precition_tolerance = 5 # in milimiters
center_point = [5, 5]
accuracy_distances = [2.4/2, 2.4/2, 3.6/2]

presition = Presition(center_point, distances, precition_tolerance, accuracy_distances)
presition.plot()