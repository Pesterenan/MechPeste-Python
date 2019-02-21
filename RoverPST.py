#!/usr/bin/env python3
# coding: "UTF-8"
# Autor: Renan Torres <pesterenan@gmail.com>
# Data: 14/02/2019
# Módulo MechRover

import krpc
import math
from time import sleep
from ControlePID import ControlePID
import Vetor


class RoverPST:

	margem_ui = 20

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
		print("Está buscando o alvo: {0:.20}".format(str(self.definir_alvo())))
		print()
		self.controle_direcao = ControlePID()
		self.controle_direcao.ajustar_pid(0.01, 0.001, 0.1)
		self.controle_aceleracao = ControlePID()
		self.controle_aceleracao.ajustar_pid(0.1, 0.001, 0.1)
		self.construir_painel_info()
		self.controlar_rover()

	def controlar_rover(self):
		while True:
			self.definir_alvo()
			self.posicao_rover = Vetor.Vetor(self.posicionar_vetor(self.rover.position(self.ponto_ref)))

			self.posicao_alvo = Vetor.Vetor(self.posicionar_vetor(self.alvo.position(self.ponto_ref)))

			self.direcao_rover = Vetor.Vetor(self.rover.direction(self.ponto_ref_orbital))
			print(self.direcao_rover, "3")

			self.distancia_entre_pontos = self.posicao_alvo.subtrai(self.posicao_rover)
			print(self.distancia_entre_pontos.normalizar())
			self.angulo_alvo = Vetor.angulo_direcao(self.distancia_entre_pontos.normalizar())
			self.angulo_rover = Vetor.angulo_direcao(self.direcao_rover)

			self.controle_direcao.entrada_pid((self.angulo_rover))

			self.controle_direcao.limite_pid(self.angulo_alvo)
			self.controle_aceleracao.entrada_pid(self.parametros_rover.horizontal_speed)
			self.controle_aceleracao.limite_pid(1)

			#print(self.controle_direcao.computar_pid(), "PID DIR")
			if not 0 < self.angulo_alvo - self.angulo_rover < 3:
				self.rover.control.wheel_steering = self.controle_direcao.computar_pid()


			else:
				self.rover.control.wheel_steering = 0

			#print(self.controle_aceleracao.computar_pid(), "PID ACEL")
			#self.acelerar_rover(self.controle_aceleracao.computar_pid())
			#print(self.distancia_entre_pontos.magnitude(), "DISTANCIA")
			self.atualizar_infos()
			sleep(0.05)

	def posicionar_vetor(self, arg):
		if type(arg) is Vetor.Vetor:
			vetor = (arg.x, arg.y, 0)
			vetor_retorno = self.centro_espacial.transform_position(vetor, self.ponto_ref, self.ponto_ref_orbital)
		else:
			vetor_retorno = self.centro_espacial.transform_position(arg, self.ponto_ref, self.ponto_ref_orbital)
		return round(vetor_retorno[0], 2), round(vetor_retorno[1], 2), round(vetor_retorno[2], 2)

	def acelerar_rover(self, arg0):
		self.rover.control.wheel_throttle = arg0

	def definir_alvo(self):
		try:
			self.alvo = self.centro_espacial.target_vessel
			return self.alvo
		except AttributeError:
			return "SEM ALVO"

	def construir_painel_info(self):
		# Tela do jogo:
		self.tela_itens = self.conexao_krpc.ui.stock_canvas
		# Tamanho da tela de jogo
		tamanho_tela = self.tela_itens.rect_transform.size
		# Adicionar um painel para conter os elementos de UI
		painel_info = self.tela_itens.add_panel(True)

		# Posicionar o painel a esquerda da tela
		painel_transform = painel_info.rect_transform
		painel_transform.size = (200.0, 100.0)
		painel_transform.position = (-int(tamanho_tela[0] * 0.25), int(tamanho_tela[1] * 0.16))

		self.txt_angulo_rover = painel_info.add_text("Angulo Rover: ", True)
		self.txt_angulo_alvo = painel_info.add_text("Angulo Alvo: ", True)
		self.txt_angulo_distancia = painel_info.add_text("Angulo Distancia: ", True)
		self.txt_distancia_alvo = painel_info.add_text("Distância até o alvo: ", True)
		self.txt_angulo_rover.rect_transform.size = (200.0, __class__.margem_ui)
		self.txt_angulo_alvo.rect_transform.size = (200.0, __class__.margem_ui)
		self.txt_angulo_distancia.rect_transform.size = (200.0, __class__.margem_ui)
		self.txt_distancia_alvo.rect_transform.size = (200.0, __class__.margem_ui)
		self.txt_angulo_rover.rect_transform.position = (__class__.margem_ui, __class__.margem_ui)
		self.txt_angulo_alvo.rect_transform.position = (__class__.margem_ui, 0)
		self.txt_angulo_distancia.rect_transform.position = (__class__.margem_ui, -__class__.margem_ui)
		self.txt_distancia_alvo.rect_transform.position = (__class__.margem_ui, -(__class__.margem_ui * 2))
		self.txt_angulo_rover.color = (0.0, 0.0, 0.0)
		self.txt_angulo_rover.size = 12
		self.txt_angulo_alvo.color = (0.0, 0.0, 0.0)
		self.txt_angulo_alvo.size = 12
		self.txt_angulo_distancia.color = (1.0, 1.0, 1.0)
		self.txt_angulo_distancia.size = 12
		self.txt_distancia_alvo.color = (1.0, 1.0, 1.0)
		self.txt_distancia_alvo.size = 12

	def atualizar_infos(self):
		self.txt_angulo_rover.content = "Angulo Rover: " + str(self.angulo_rover)
		self.txt_angulo_alvo.content = "Angulo Alvo: " + str(self.angulo_alvo)
		self.txt_angulo_distancia.content = "Angulo Distância: " + str(Vetor.angulo_direcao(self.distancia_entre_pontos))
		self.txt_distancia_alvo.content = "Distância: " + str(round(self.distancia_entre_pontos.magnitude(), 1)) + "m"

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


