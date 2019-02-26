#!/usr/bin/env python3
# coding: "UTF-8"
# Autor: Renan Torres <pesterenan@gmail.com>
# Data: 07/02/2019
# Módulo Vetor

import math


class Vetor:
	x = 0
	y = 0
	direcao = 0

	def __init__(self, arg0=None, arg1=None):
		if type(arg0) == tuple:
			self.x = round(arg0[0], 3)
			self.y = round(arg0[1], 3)
		else:
			self.x = round(arg0, 3)
			self.y = round(arg1, 3)

	def __str__(self):
		return "(" + str(round(self.x, 3)) + "," + str(round(self.y, 3)) + ")"

	def inverte(self):
		return Vetor(self.y, self.x)

	def magnitude(self):
		return math.sqrt(self.x ** 2 + self.y ** 2)

	def normalizar(self):
		m = self.magnitude()
		if m != 0:
			return Vetor(self.x / m, self.y / m)
		return Vetor(self.x, self.y)

	def limitar(self, maxi):
		if self.magnitude() > maxi:
			self.normalizar()
			self.multiplica(maxi)


	def soma(self, outro):
		return Vetor(self.x + outro.x, self.y + outro.y)

	def subtrai(self, outro):
		return Vetor(self.x - outro.x, self.y - outro.y)

	def multiplica(self, escalar):
		return Vetor(self.x * escalar, self.y * escalar)

	def divide(self, escalar):
		if escalar != 0:
			return Vetor(self.x / escalar, self.y / escalar)
		return Vetor(0, 0)

	def angulo_direcao(self):
		self.direcao = (math.atan2(self.y, self.x) / math.pi) * 180
		return self.direcao


def vetor_distancia(vetor_a, vetor_b):
	return Vetor(-(vetor_a[2]) + (vetor_b[2]), -(vetor_a[1]) + (vetor_b[1]))


def angulo_direcao(vetor):
	direcao = (math.atan2(vetor.y, vetor.x) / math.pi) * 180
	return round(direcao, 2)

