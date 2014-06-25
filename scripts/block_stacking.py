#!/usr/bin/env python

import rospy
import baxter_interface
import std_msgs.msg

class InvalidBlockSpaceError(Exception):
	def __init__(self):
		self.error_message = "InvalidBlockSpaceError: Block parent needs to be an instance of StackSpace."
		rospy.logerror(error_message)
	
	def __str__(self):
		return self.error_message

class Block():
	"""Basic block structure that will be identified and manipulated by Baxter. \
	Maybe possible to adjoint multiple Block structures to create different types of \
	shapes."""
	
	def __init__():
		self.dimensions = (0.0, 0.0, 0.0); # x, y, z
		self.parent = None
		self.position = None
	
	def set_parent(self, parent):
		"""Takes a BlockSpace as a container for the Block"""
		if isinstance(parent, StackSpace): self.parent = parent
		else: raise InvalidBlockSpaceError
	
	def set_position(self, position):
		"""Takes a 3-tuple as the position variable"""
		if position is tuple and len(position) is 2 and all([type(k) is float or int for k in position]):
			# Transform the check into helper function.
			self.position = position
		else: raise TypeError
	

class BlockSpace():
	"""Container for stable, grounded, base blocks ontop of which other levels of \
	Blocks can be stacked."""
	
	def __init__(self):
		self.blocks = []	# List of Blocks
		self.dimensions = (0.0, 0.0)	# 2-dimensional planar space
		self.occupied_base_space = 0.0	# available_space = total_space - space_taken
	
	def find_free_position(self):
		

	def insert_block(self, block):
		block.set_parent(self)
		if is_free_position(block.position):
			self.blocks.append(block)
		else:
			free_position = find_free_position()
			if free_position is not None: block.set_position(free_position)
			
