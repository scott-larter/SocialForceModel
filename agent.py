# -*-coding:utf-8-*-
# Author: Scott Larter

import numpy as np
from tools import *
import random

class Agent(object):
	def __init__(self, agent_id, group_id, subgroup_id, x=1, y=1):
		self.agentId = agent_id
		self.groupId = group_id
		self.subgroupId = subgroup_id

		# walls.csv setup
		if self.groupId % 2 == 0:
			self.posX = random.uniform(100,107)
			self.posY = random.uniform(6,29)
			self.destX = random.uniform(8,15)
			self.destY = random.uniform(18,29)
		else:
			self.posX = random.uniform(8,15)
			self.posY = random.uniform(6,29)
			self.destX = random.uniform(100,107)
			self.destY = random.uniform(6,17)

		self.destY = 17.5

		# walls2.csv setup

		"""self.posX = random.uniform(100,107)

		if self.agentId < 6:
			self.posY = ((self.agentId % 6) + 1) * 8
		else:
			self.posY = (((self.agentId % 6) + 1) * 8) + 4
		self.destX = random.uniform(8,15)
		self.destY = 32.5"""

		if self.subgroupId % 2 == 0:
			self.subgroupColor = [0, 255, 0]
		else:
			self.subgroupColor = [255, 255, 0]

		# set agent colour based on group
		if self.groupId == 0:
			self.color = [255, 0, 0]
		elif self.groupId == 1:
			self.color = [0, 0, 255]
		elif self.groupId == 2:
			self.color = [0, 255, 0]
		else:
			self.color = [0, 0, 0]

		self.visAngle = np.pi / 2.0

		self.pos = np.array([self.posX, self.posY])

		self.actualVX = 0 #random.uniform(0,1.6)
		self.actualVY = 0 #random.uniform(0,1.6)
		self.actualV = np.array([self.actualVX, self.actualVY])

		self.dest = np.array([self.destX, self.destY])
		self.direction = normalize(self.dest - self.pos)

		#self.desiredSpeed = random.uniform(0.75, 1.0)
		self.desiredSpeed = 1.25
		self.desiredV = self.desiredSpeed*self.direction

		self.acclTime = random.uniform(8,16) #10.0
		self.drivenAcc = (self.desiredV - self.actualV) / self.acclTime

		self.mass = random.uniform(50,100) #60
		self.radius = 1 #1.6
		self.interactionRange = 3
		self.p = 0.6

		self.bodyFactor = 120000
		self.slideFricFactor = 240000
		self.A = 1
		self.B = 0.8 #random.uniform(0.8,1.6) #0.8 #0.08

		self.Goal = 0
		self.timeOut = 0.0

	
	def adaptVel(self):
		deltaV = self.desiredV - self.actualV

		# if deltaV is close enough to zero, assign value zero
		if np.allclose(deltaV, np.zeros(2)):
			deltaV = np.zeros(2)

		return (deltaV * self.mass) / self.acclTime


	def peopleInteraction(self, other, A=1, B=0.8):
		rij = self.radius + other.radius
		dij = np.linalg.norm(self.pos - other.pos)
		nij = (self.pos - other.pos) / dij
		tij = np.array([-nij[1], nij[0]])
		deltaVji = (other.actualV - self.actualV)*tij

		fij = A*np.exp((rij-dij)/B)*nij*20
		+ self.bodyFactor*g(rij-dij)*nij*1000000
		+ self.slideFricFactor*g(rij-dij)*deltaVji*tij

		return fij


	"""def wallInteraction(self, wall):
		ri = self.radius
		diw,niw = distanceP2W(self.pos, wall)
		first = -self.A*np.exp((ri-diw)/self.B)*niw*160
		+ self.bodyFactor*g(ri-diw)*niw/10000
		#tiw = np.array([-niw[1],niw[0]])
		#second = self.slideFricFactor*g(ri-diw)*(self.actualV*tiw)*tiw
		return first #- second"""

	def wallInteraction(self, wall, A=1, B=0.8):
		ri = self.radius
		diw,niw = distanceP2W(self.pos, wall)
		tiw = np.array([-niw[1],niw[0]])
		fiw = -A*np.exp((ri-diw)/B)*niw*160
		+ self.bodyFactor*g(ri-diw)*niw/10000
		- self.slideFricFactor*g(ri-diw)*(self.actualV*tiw)*tiw

		return fiw


	def groupVisual(self, group, Beta1 = 4):
		Ci = centerOfMass(group)

		distAi2Di = np.linalg.norm(self.pos - self.dest)
		distAi2Ci = np.linalg.norm(self.pos - Ci)
		distDi2Ci = np.linalg.norm(self.dest - Ci)
		
		alpha = np.arccos((np.power(distAi2Di, 2)
		+ np.power(distAi2Ci, 2) - np.power(distDi2Ci, 2))
		/ (2 * distAi2Di * distAi2Ci)) - self.visAngle
		
		f_vis = -Beta1 * alpha * self.actualV

		return f_vis


	def groupAttraction(self, group, Beta2 = 1.5):
		Ci = centerOfMass(group)
		Ui = normalize(Ci - self.pos)

		thresholdDist = (len(group) - 1) / 2
		if np.linalg.norm(Ci - self.pos) > thresholdDist:
			qA = 1
		else:
			qA = 0

		f_att = qA * Beta2 * Ui

		return f_att


	def ownGroupRepulsion(self, other, Beta3 = 1.5):
		Wik = normalize(other.pos - self.pos)
		distAi2Aj = np.linalg.norm(other.pos - self.pos)
		safetyDist = 0.5

		thresholdDist = (2 * self.radius) + safetyDist

		if distAi2Aj <= thresholdDist:
			qR = 1
		else:
			qR = 0

		f_rep = qR * Beta3 * Wik

		return f_rep


	def otherGroupRepulsion(self, group, A = 500, B = 1):
		# center of mass of other group
		c = centerOfMass(group)

		# check if ped is past group
		if np.dot(self.direction, c - self.pos) <= 0:
			print("here")
			return np.array([0.0, 0.0])

		# distance from ped i to center of mass
		dij = np.linalg.norm(c - self.pos)

		# find furthest group member
		maxDist = 0
		memberRadius = 0
		for member in group:
			memberDist = np.linalg.norm(c - member.pos)
			if memberDist > maxDist:
				maxDist = memberDist
				memberRadius = member.radius

		# rij = radius of ped i + radius of group
		rij = self.radius + memberRadius + maxDist

		# vertical component of walking dir of ped i
		ev1 = np.array([-self.direction[1], self.direction[0]])
		ev2 = np.array([self.direction[1], -self.direction[0]])

		if np.dot(c - self.pos, ev1) > 0:
			ev = ev1
		else:
			ev = ev2

		cosAlpha = np.dot(c - self.pos, ev) / (np.linalg.norm(c - self.pos) * np.linalg.norm(ev))

		S = g(1 - ((dij * cosAlpha) / rij))

		f_rep = A * np.exp((rij - dij) / B) * S * ev

		return f_rep


	def subgroupForces(self, group, M = 1, N = 1, K = 1):
		f_att = np.array([0.0, 0.0])
		f_dist = np.array([0.0, 0.0])

		for sid,sub in enumerate(group):
			if sid != self.subgroupId:
				ownC = centerOfMass(group[self.subgroupId])
				otherC = centerOfMass(sub)
				otherDestDist = np.linalg.norm(self.dest - otherC)
				ownDestDist = np.linalg.norm(self.dest - self.pos)

				if otherDestDist < ownDestDist:
					# direction and distance between centers of subgroups
					el = normalize(otherC - ownC)
					l = np.linalg.norm(otherC - ownC)

					# average walking direction of own group
					groupDir = np.array([0.0, 0.0])
					for member in group[self.subgroupId]:
						groupDir += member.direction
					groupDir = normalize(groupDir)

					# vertical walking direction of ped
					ew1 = np.array([-groupDir[1], groupDir[0]])
					ew2 = np.array([groupDir[1], -groupDir[0]])
					if np.dot(otherC - ownC, ew1) > 0:
						ew = ew1
					else:
						ew = ew2

					# angle between el and ew
					cosBeta = np.dot(el, ew) / (np.linalg.norm(el) * np.linalg.norm(ew))

					f_att += M * l * cosBeta * ew
					f_dist += N * np.log(l / K) * el

		return f_att + f_dist

	#def wallOnRoute(self, wall):
	#self.pos
	#self.actualV
	#return true