#!/usr/bin/env python3

# Autor: Renan Torres <pesterenan@gmail.com>
# Data: 31/08/2018
# Módulo SuicideBurn

# Imports necessários
from time import sleep
import krpc
from ControlePID import ControlePID
from Navegacao import Navegacao

"""MechPestePython - Módulo de Suicide Burn"""


class SuicideBurn:
	margem_ui = 20
	conexao = None
	centro_espacial = None

	def __init__(self, conexao):
		__class__.conexao = conexao
		__class__.centro_espacial = conexao.space_center
		self.nave_atual = __class__.centro_espacial.active_vessel # objeto da nave
		self.ponto_ref = self.nave_atual.orbit.body.reference_frame
		self.voo_nave = self.nave_atual.flight(self.ponto_ref)
		# Parâmetros da nave
		self.altitude_da_nave = conexao.add_stream(getattr, self.voo_nave, "surface_altitude")
		self.vel_vert_nave = conexao.add_stream(getattr, self.voo_nave, "vertical_speed")
		self.massa_total_nave = conexao.add_stream(getattr, self.nave_atual, "mass")
		self.ut = conexao.add_stream(getattr, __class__.centro_espacial, "ut")
		self.controle_acel = ControlePID()
		self.controle_pouso = ControlePID()
		self.nav = None
		self.nav = Navegacao()
		self.distancia_pouso = 30  # Altura pra começar o Hover
		self.forca_grav = self.nave_atual.orbit.body.surface_gravity
		self.exec_suicide_burn = False
		self.pode_pousar = False
		self.nave_twr_max = 1
		self.acel_max = 0
		self.empuxo_total = 0
		self.distancia_ate_queima = 0
		self.tempo_da_queima = 0
		self.correcao = 0
		self.construirPainelInfo()

	def iniciar_suicide_burn(self, conexao):
		print("Nave Atual: ", self.nave_atual.name)
		print(str(self.nave_atual.situation))
		print("Força da Gravidade Atual: ", self.forca_grav, "Corpo Celeste: ", self.nave_atual.orbit.body.name)
		self.calcular_parametros()
		print("Força de TWR da Nave: ", self.nave_twr_max)
		self.controle_acel.tempo_amostragem(40)
		self.controle_acel.ajustar_pid()  # <== AJUSTES PID
		self.controle_acel.limitar_saida(0, 1)
		self.controle_pouso.tempo_amostragem(40)
		self.controle_pouso.ajustar_pid(0.5, 0.001, 1)  # <== AJUSTES PID
		self.controle_pouso.limitar_saida(0, 1)
		self.controle_pouso.limite_pid(5)
		self.nave_atual.auto_pilot.engage()  # LIGAR O PILOTO
		self.decolagem_de_teste()
		self.nav.navegacao(__class__.centro_espacial, self.nave_atual)
		while not self.exec_suicide_burn:  # LOOP esperando para executar o SuicideBurn
			self.calcular_parametros()
			self.pode_pousar = False
			self.nav.mirar_nave()
			# print("Distância Até Queima: {0:>20.1f}{1}".format(self.distancia_ate_queima, "m"))
			# print("Dist até Suicide: {0:>20.1f}{1}".format(
			# 	(self.altitude_da_nave() - self.distancia_ate_queima - self.distancia_pouso), "m"))
			sleep(0.1)
			# Ativa freios e RCS para melhorar o controle da descida:
			if 100 < self.altitude_da_nave() < 2000:
				self.nave_atual.control.brakes = True
			else:
				self.nave_atual.control.brakes = False
			if self.altitude_da_nave() < 3000:
				self.nave_atual.control.rcs = True
			if (0 > self.altitude_da_nave() - self.distancia_ate_queima - self.distancia_pouso) and (self.vel_vert_nave() < -1):
				self.exec_suicide_burn = True

		# -=-=- Loop Principal do Suicide Burn -=-=-
		while self.exec_suicide_burn:

			self.calcular_parametros()  # atualiza valores
			# print("Controle Acel PID: {0:>10.2f}".format(self.controle_acel.computar_pid()))
			# print("Distância Até Queima: {0:>10.1f}{1}".format(self.distancia_ate_queima, "m"))
			# print("Dist até Suicide: {0:>10.1f}{1}".format((self.altitude_da_nave() - self.distancia_ate_queima - self.distancia_pouso), "m"))
			# Descer pernas de pouso da nave em menos de 100 metros
			if self.altitude_da_nave() < 100:
				self.nave_atual.control.gear = True

			# Apontar nave para o retrograde se a velocidade horizontal for maior que 1m/s
			print("Vel Horizontal: ",self.voo_nave.horizontal_speed)
			if self.voo_nave.horizontal_speed > 1:
				self.nav.mirar_nave()
				self.pode_pousar = False
			else:
				self.nave_atual.control.target_pitch = 0
				self.pode_pousar = True

			# -=- Informa ao PID a altitude da nave e o limite -=-
			self.controle_acel.entrada_pid(self.altitude_da_nave())
			self.controle_acel.limite_pid(self.distancia_ate_queima + (self.distancia_pouso*0.50))
			self.controle_acel.limitar_saida(0, 1)
			# -=- Corrigir a aceleração -=-
			if not self.pode_pousar:
				correcao_anterior = self.nave_atual.control.throttle
				try:
					self.aceleracao(float(self.controle_acel.computar_pid()))
					print("Valor Saída ACEL: ", self.controle_acel.computar_pid())
				except ZeroDivisionError:
					print("Erro no cálculo. Usando valor antigo")
					self.aceleracao(correcao_anterior)
			else:
				# Informar valor da velocidade absoluto
				self.controle_pouso.entrada_pid(abs(self.vel_vert_nave()))
				correcao_anterior = self.nave_atual.control.throttle
				try:
					self.aceleracao(abs((1.5 - self.controle_pouso.computar_pid()) / self.nave_twr_max))
					print("Valor Saída POUSO: ", self.controle_pouso.computar_pid())
				except ZeroDivisionError:
					print("Erro no cálculo. Usando valor antigo")
					self.aceleracao(correcao_anterior)

			# -=-=- Terminando o pouso -=-=-
			if self.terminar_pouso():
				self.nave_atual.auto_pilot.disengage()  # DESLIGAR O PILOTO
				self.exec_suicide_burn = False

			sleep(0.04)

	def calcular_parametros(self):
		# Calcular TWR:
		try:
			self.nave_twr_max = self.nave_atual.available_thrust / (self.massa_total_nave() * self.forca_grav)
			# Aceleração Máxima da Nave:
			self.acel_max = (self.nave_twr_max * self.forca_grav) - self.forca_grav
			# Tempo de queima dessa aceleração e distancia até iniciar a queima de suicide burn com essa aceleração:
			self.tempo_da_queima = self.vel_vert_nave() / self.acel_max
			self.distancia_ate_queima = (self.vel_vert_nave() * self.tempo_da_queima + 1 / 2 * self.acel_max * (self.tempo_da_queima ** 2))
		except:
			self.nave_twr_max = 1
			self.tempo_da_queima = 0
			self.distancia_ate_queima = 0
		self.controle_acel.ajustar_pid(self.nave_twr_max/1000, 0.001, 1)
		self.controle_pouso.ajustar_pid(self.nave_twr_max / 100, 0.001, 1)
		# Mostrar dados:
		try:
			self.txt_altitude.content = str("Altitude: {0:.1f}{1}".format(self.altitude_da_nave(), "m"))
			self.txt_velocidade.content = str("Velocidade: {0:.1f}{1}".format(self.vel_vert_nave(), "m/s"))
			self.txt_suicide_dist.content = str("Distância até Suicide: {0:>20.1f}{1}".format(
				(self.altitude_da_nave() - self.distancia_ate_queima - self.distancia_pouso), "m"))
		except:
			self.txt_altitude.content = "Impossível buscar altitude"
			self.txt_velocidade.content = "Impossível buscar velocidade"
			self.txt_suicide_dist.content = "Impossível buscar distância"
		try:
			for motor in self.nave_atual.parts.engines:
				self.empuxo_total += motor.max_thrust
			twr_atual = float(self.empuxo_total / (self.massa_total_nave() * self.forca_grav))
			self.txt_twr.content = str("TWR: {0:.1f}".format(twr_atual))
			self.empuxo_total = 0
		except ZeroDivisionError:
			self.txt_twr.content = "Impossível buscar TWR"

	def aceleracao(self, valor):
		""" Acelera a nave de acordo com o valor
		Se o valor for negativo, o resultado é 0, se for maior que 1, fica 1"""
		valor = valor if valor > 0 else 0
		valor = valor if valor < 1 else 1
		self.nave_atual.control.throttle = valor

	def decolagem_de_teste(self):
		situacao_da_nave = str(self.nave_atual.situation)
		# Se o veículo está pousado ou na base de lançamento, ele sobe:
		if situacao_da_nave == "VesselSituation.landed" or situacao_da_nave == "VesselSituation.pre_launch":
			if situacao_da_nave == "VesselSituation.pre_launch":
				self.nave_atual.control.activate_next_stage()
			self.nave_atual.control.gear = False
			self.calcular_parametros()
			self.aceleracao(1)
			while self.altitude_da_nave() <= self.distancia_pouso:
				self.nave_atual.control.target_pitch = 0
				sleep(0.1)
			self.aceleracao(0)
			sleep(0.1)
			self.nave_atual.control.target_pitch = 0  # mirar pra cima <<<
		else:
			self.aceleracao(0)

	def terminar_pouso(self):
		situacao_da_nave = str(self.nave_atual.situation)
		if situacao_da_nave == "VesselSituation.landed" or situacao_da_nave == "VesselSituation.splashed" and self.pode_pousar == True:
			while self.nave_atual.control.throttle > 0.1:
				acel = self.nave_atual.control.throttle - 0.3
				self.aceleracao(acel)
				sleep(0.1)
			print("Pouso finalizado.")
			self.aceleracao(0)
			self.nave_atual.auto_pilot.disengage()
			self.nave_atual.control.rcs = True
			self.nave_atual.control.sas = True
			self.nave_atual.control.brakes = False
			self.pode_pousar = False
			del self.nav
			return True
		return False

	def construirPainelInfo(self):
		# Tela do jogo:
		self.tela_itens = __class__.conexao.ui.stock_canvas
		# Tamanho da tela de jogo
		tamanho_tela = self.tela_itens.rect_transform.size
		# Adicionar um painel para conter os elementos de UI
		painel_info = self.tela_itens.add_panel(True)

		# Posicionar o painel a esquerda da tela
		painel_transform = painel_info.rect_transform
		painel_transform.size = (400.0, 100.0)
		painel_transform.position = (-int(tamanho_tela[0] * 0.25), int(tamanho_tela[1] * 0.16))

		self.txt_altitude = painel_info.add_text("Altitude: ", True)
		self.txt_velocidade = painel_info.add_text("Velocidade: ", True)
		self.txt_twr = painel_info.add_text("TWR: ", True)
		self.txt_suicide_dist = painel_info.add_text("Distância Suicide: ", True)
		self.txt_altitude.rect_transform.size = (400.0, __class__.margem_ui)
		self.txt_velocidade.rect_transform.size = (400.0, __class__.margem_ui)
		self.txt_twr.rect_transform.size = (400.0, __class__.margem_ui)
		self.txt_suicide_dist.rect_transform.size = (400.0, __class__.margem_ui)
		self.txt_altitude.rect_transform.position = (__class__.margem_ui, __class__.margem_ui)
		self.txt_velocidade.rect_transform.position = (__class__.margem_ui, 0)
		self.txt_twr.rect_transform.position = (__class__.margem_ui, -__class__.margem_ui)
		self.txt_suicide_dist.rect_transform.position = (__class__.margem_ui, -(__class__.margem_ui * 2))
		self.txt_altitude.color = (0.0, 0.0, 0.0)
		self.txt_altitude.size = 12
		self.txt_velocidade.color = (0.0, 0.0, 0.0)
		self.txt_velocidade.size = 12
		self.txt_twr.color = (1.0, 1.0, 1.0)
		self.txt_twr.size = 12
		self.txt_suicide_dist.color = (1.0, 1.0, 1.0)
		self.txt_suicide_dist.size = 12

		# Adicionar botões:
		bt_sburn = painel_info.add_button("Suicide Burn", True)
		bt_cancelar = painel_info.add_button("Cancelar", True)
		bt_sburn.rect_transform.position = (100.0, -__class__.margem_ui)
		bt_cancelar.rect_transform.position = (100.0, __class__.margem_ui)

		# stream para checar se o botao foi clicado
		bt_sburn_clk = __class__.conexao.add_stream(getattr, bt_sburn, "clicked")
		bt_cancelar_clk = __class__.conexao.add_stream(getattr, bt_cancelar, "clicked")
		cancelar = False
		# Esperar clique do botão:
		while not (bt_sburn_clk()):
			try:
				self.txt_altitude.content = str("Altitude: {0:.1f}{1}".format(self.altitude_da_nave(),"m"))
				self.txt_velocidade.content = str("Velocidade: {0:.1f}{1}".format(self.vel_vert_nave(),"m/s"))
			except:
				self.txt_altitude.content = "Impossível buscar altitude"
				self.txt_velocidade.content = "Impossível buscar velocidade"
			try:
				for motor in self.nave_atual.parts.engines:
					self.empuxo_total += motor.max_thrust
				twr_atual = float(self.empuxo_total / (self.massa_total_nave() * self.forca_grav))
				self.txt_twr.content = str("TWR: {0:.1f}".format(twr_atual))
				self.empuxo_total = 0
			except:
				self.txt_twr.content = "Impossível buscar TWR"
			sleep(0.5)
			if bt_cancelar_clk():
				cancelar = True
				break
		if not cancelar:
			self.iniciar_suicide_burn(conexao)
			bt_sburn.clicked = False
			bt_sburn.remove()


if __name__ == "__main__":
	conexao = krpc.connect(name="MechPeste")
	sb = SuicideBurn(conexao)
