#!/usr/bin/env python3
# coding: "UTF-8"
# Autor: Renan Torres <pesterenan@gmail.com>
# Data: 14/02/2019
# Módulo MechRover

import krpc
import math
from time import sleep
import ControlePID
import Vetor


class RoverPST:

	def __init__(self, arg0):
		self.conexao_krpc = arg0
		self.centro_espacial = arg0.space_center
		self.marcadores = self.centro_espacial.waypoint_manager  # Marcadores no mapa
		self.rover = self.centro_espacial.active_vessel
		self.ponto_ref_orbital = self.rover.orbit.body.reference_frame
		self.ponto_ref = self.rover.surface_reference_frame
		self.parametros_rover = self.rover.flight(self.ponto_ref)

		print("O nome do Rover é: {0:.20}".format(self.rover.name))
		self.alvo = self.centro_espacial.target_vessel
		print("Está buscando o alvo: {0:.20}".format(self.alvo.name))
		print()
		self.controle_direcao = ControlePID.ControlePID()
		self.controle_direcao.ajustar_pid(0.01, 0.001, 0.1)
		self.controle_aceleracao = ControlePID.ControlePID()
		self.controle_aceleracao.ajustar_pid(0.1, 0.001, 0.1)

		self.controlar_rover()

	def controlar_rover(self):
		while True:
			self.alvo = self.centro_espacial.target_vessel
			self.posicao_rover = Vetor.Vetor(self.posicionar_vetor(self.rover.position(self.ponto_ref)))
			self.posicao_alvo = Vetor.Vetor(self.posicionar_vetor(self.alvo.position(self.ponto_ref)))
			self.direcao_rover = Vetor.Vetor(self.rover.direction(self.ponto_ref_orbital))
			self.distancia_entre_pontos = self.posicao_alvo.subtrai(self.posicao_rover)
			self.angulo_alvo = Vetor.angulo_direcao(self.distancia_entre_pontos)
			self.angulo_rover = Vetor.angulo_direcao(self.direcao_rover)

			self.controle_direcao.entrada_pid(self.angulo_rover)
			self.controle_direcao.limite_pid(self.angulo_alvo)
			self.controle_aceleracao.entrada_pid(self.parametros_rover.horizontal_speed)
			self.controle_aceleracao.limite_pid(15)

			#print(self.controle_direcao.computar_pid(), "PID DIR")
			if not 0 < self.angulo_alvo - self.angulo_rover < 3:
				self.rover.control.wheel_steering = self.controle_direcao.computar_pid()
			else:
				self.rover.control.wheel_steering = 0
			self.rover.control.wheel_throttle = self.controle_aceleracao.computar_pid()
			#print(self.controle_aceleracao.computar_pid(), "PID ACEL")

			#print(self.distancia_entre_pontos.magnitude(), "DISTANCIA")
			print(self.angulo_alvo, "ANGULO ALVO")
			print(self.angulo_rover, "ANGULO ROVER")
			self.rover.auto_pilot.disengage()
			sleep(0.04)

	def posicionar_vetor(self, arg):
		if type(arg) is Vetor.Vetor:
			vetor = (arg.x, arg.y, 0)
			vetor_retorno = self.centro_espacial.transform_position(vetor, self.ponto_ref, self.ponto_ref_orbital)
		else:
			vetor_retorno = self.centro_espacial.transform_position(arg, self.ponto_ref, self.ponto_ref_orbital)
		return round(vetor_retorno[0], 2), round(vetor_retorno[1], 2), round(vetor_retorno[2], 2)


if __name__ == "__main__":
	conexao = krpc.connect(name="RoverPST")
	mr = RoverPST(conexao)


# REFERENCE_FRAME = ORIENTADO E POSICIONADO COM A NAVE
# O EIXO X É PRA DIREITA
# O EIXO Y É PARA FRENTE >>
# O EIXO Z É PARA BAIXO

# SURFACE_REFERENCE_FRAME = ORIENTADO PELA SUPERFICIE E POSICIONADO COM A NAVE
# O EIXO X É PRO ZENITE (DO CENTRO PRA CIMA)
# O EIXO Y É PRO NORTE (BUSSOLA)
# O EIXO Z É PRO LESTE

# ORBITAL_REFERENCE_FRAME = RELATIVO A NAVE ORIENTADO A ORBITA
# O EIXO X É PRO CENTRO
# O EIXO Y É PRA FRENTE
# O EIXO Z É PRO NORTE


