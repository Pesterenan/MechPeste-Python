#!/usr/bin/env python3
# coding: "UTF-8"
# Autor: Renan Torres <pesterenan@gmail.com>
# Data: 14/02/2019
# Módulo MechRover

import krpc
from time import sleep
from ControlePID import ControlePID
import Vetor


class RoverPST:

	margem_ui = 20

	def __init__(self, arg0):
		self.conexao_krpc = arg0  # Conexão com o KRPC
		self.centro_espacial = arg0.space_center  # Objeto do Centro Espacial
		self.marcadores = self.centro_espacial.waypoint_manager  # Gerenciador de Marcadores no mapa
		self.rover = self.centro_espacial.active_vessel  # Objeto do Rover (Nave Atual)
		self.ponto_ref_orbital = self.rover.orbit.body.reference_frame  # Ponto de Referência Orbital
		self.ponto_ref_superficie = self.rover.surface_reference_frame  # Ponto de Referência da Superfície
		self.ponto_ref_rover = self.rover.reference_frame  # Ponto de Referência do Rover
		self.parametros_rover = self.rover.flight(self.ponto_ref_orbital)  # Parâmetros de "voo" do Rover
		# Definindo Vetores de posição e direção:
		self.posicao_rover = Vetor.Vetor(0, 0)
		self.posicao_alvo = Vetor.Vetor(0, 0)
		self.direcao_rover = Vetor.Vetor(0, 0)
		self.distancia_entre_pontos = Vetor.Vetor(0, 0)
		# Variáveis de Angulo:
		self.angulo_alvo = 0.0
		self.angulo_rover = 0.0

		print("O nome do Rover é: {0:.30}".format(self.rover.name))
		self.alvo = self.definir_alvo()
		if self.alvo is not None:
			print("Está buscando o alvo: {0:.30}".format(str(self.alvo.name)), "\n")

		self.ctrl_direcao = ControlePID()
		self.ctrl_direcao.ajustar_pid(0.01, 0.001, 0.01)
		self.ctrl_aceleracao = ControlePID()
		self.ctrl_aceleracao.ajustar_pid(0.5, 0.001, 0.1)
		self.ctrl_distancia_alvo = ControlePID()
		self.ctrl_distancia_alvo.ajustar_pid(0.1, 0.001, 5)
		self.ctrl_distancia_alvo.limitar_saida(-1, 1)
		self.limite_distancia_alvo = 50
		self.velocidade_maxima = 15
		self.velocidade_curva = 3
		self.construir_painel_info()
		self.controlar_rover()

	def controlar_rover(self):
		while True:
			try:
				self.definir_vetor_direcao(self.definir_alvo())
				self.informar_ctrl_pid()
				# Com tudo calculado, pilotar o rover:
				if self.distancia_entre_pontos.magnitude() > self.limite_distancia_alvo:
					self.acelerar_rover(self.ctrl_aceleracao.computar_pid())
					self.pilotar_rover()
				else:
					self.acelerar_rover(self.ctrl_aceleracao.computar_pid() * self.ctrl_distancia_alvo.computar_pid())

				self.atualizar_infos()
			except AttributeError:
				print("Sem alvo selecionado")
				sleep(1)

			sleep(0.05)

	def informar_ctrl_pid(self):
		# Informar ao PID de controle de direção os ângulos e tentar zerar a diferença entre eles:
		self.ctrl_direcao.entrada_pid(self.angulo_rover - abs(self.angulo_alvo))
		self.ctrl_direcao.limite_pid(0)
		# Informar ao PID de aceleração a velocidade horizontal e limitar:
		self.ctrl_aceleracao.entrada_pid(self.parametros_rover.horizontal_speed)
		self.ctrl_distancia_alvo.entrada_pid(self.limite_distancia_alvo - self.distancia_entre_pontos.magnitude())
		self.ctrl_distancia_alvo.limite_pid(0)
		diferenca_angulo = self.angulo_rover - abs(self.angulo_alvo)
		if abs(diferenca_angulo) > 30:
			self.ctrl_aceleracao.limite_pid(self.velocidade_curva)
		else:
			self.ctrl_aceleracao.limite_pid(self.velocidade_maxima)

	def definir_vetor_direcao(self, alvo):
		# Vetor de posição do Rover em relacão à superfície:
		self.posicao_rover = Vetor.Vetor(self.posicionar_vetor(self.rover.position(self.ponto_ref_superficie)))
		# Vetor de posição do Alvo em relacão ao rover e a órbita:
		self.posicao_alvo = Vetor.Vetor(self.posicionar_vetor(alvo.position(self.ponto_ref_rover)))
		# Vetor de posição do alvo em relacão à superfície e ao rover:
		self.direcao_rover = Vetor.Vetor(alvo.position(self.ponto_ref_rover))
		# Vetor de distância do alvo em relacão rover:
		self.distancia_entre_pontos = self.posicao_rover.subtrai(self.posicao_alvo)
		# Calculando angulos até o alvo e da direção do rover
		self.angulo_alvo = Vetor.angulo_direcao(self.distancia_entre_pontos)
		self.angulo_rover = Vetor.angulo_direcao(self.direcao_rover)

	def pilotar_rover(self):
		if not 0 < self.angulo_alvo - self.angulo_rover < 1:
			self.rover.control.wheel_steering = -self.ctrl_direcao.computar_pid()
		else:
			self.rover.control.wheel_steering = 0

	def posicionar_vetor(self, arg):
		if type(arg) is Vetor.Vetor:
			vetor = (arg.x, arg.y, 0)
			vetor_retorno = self.centro_espacial.transform_position(vetor, self.ponto_ref_superficie, self.ponto_ref_orbital)
		else:
			vetor_retorno = self.centro_espacial.transform_position(arg, self.ponto_ref_superficie, self.ponto_ref_orbital)
		return round(vetor_retorno[0], 2), round(vetor_retorno[1], 2), round(vetor_retorno[2], 2)

	def acelerar_rover(self, arg0):
		self.rover.control.wheel_throttle = arg0

	def definir_alvo(self):
		try:
			self.alvo = self.centro_espacial.target_vessel
			return self.alvo
		except AttributeError:
			return None

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
		self.txt_angulo_rover.content = "Correção de Ângulo: " + str(round(self.angulo_rover - abs(self.angulo_alvo),2))
		self.txt_angulo_alvo.content = "Acelerador: " + str(round(self.ctrl_aceleracao.computar_pid(),2))
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


