#!/usr/bin/python3

# Autor: Renan Torres <pesterenan@gmail.com>
# Data: 31/08/2018
# Módulo SuicideBurn

import krpc
import math
from time import sleep
from ControlePID import ControlePID
from Navegacao import Navegacao

"""MechPestePython - Módulo de Suicide Burn"""

class SuicideBurn():
	margem_ui = 20
	conexao = None
	centro_espacial = None

	def __init__(self, conexao):
		__class__.conexao = conexao
		__class__.centro_espacial = conexao.space_center
		self.nave_atual = __class__.centro_espacial.active_vessel # objeto da nave
		self.ponto_ref = self.nave_atual.orbit.body.reference_frame
		self.voo_nave = self.nave_atual.flight(self.ponto_ref)
		#Parâmetros da nave
		self.alt_nave = conexao.add_stream(getattr, self.voo_nave, "surface_altitude")
		self.vel_vert_nave = conexao.add_stream(getattr, self.voo_nave, "vertical_speed")
		self.massa_total_nave = conexao.add_stream(getattr, self.nave_atual, "mass")
		self.ut = conexao.add_stream(getattr, __class__.centro_espacial, "ut")
		
		self.controle_acel = ControlePID()
		self.controle_pouso = ControlePID()
		self.nav = Navegacao()
		self.alt_flutuacao = 30 # Altura pra começar o Hover
		self.forca_grav = self.nave_atual.orbit.body.surface_gravity
		
		self.exec_suicide_burn = False
		self.pode_pousar = False
		
		self.nave_twr_max = 1
		self.acel_max = 0
		self.empuxo_total = 0
		self.distancia_ate_queima = 0
		self.tempo_da_queima = 0
		self.nova_acel = 0

		self.construir_painel_info()

		
	
	def suicide_burn(self, conexao):
		print("Nave Atual: ", self.nave_atual.name)
		
		self.distancia_pouso = self.alt_flutuacao
		self.atualizar_variaveis()
		
		self.controle_acel.set_tempo_amostra(15)
# 		self.controle_acel.ajustar_pid(0.025, 0.001, 1) # <== AJUSTES PID
		self.controle_acel.ajustar_pid(0.025, 0.001, 1) # <== AJUSTES PID
		try:
			self.controle_acel.limitar_saida(0.8/self.nave_twr_max, self.nave_twr_max)
		except:
			self.controle_acel.limitar_saida(0,1)
			
		self.controle_pouso.set_tempo_amostra(20)
		self.controle_pouso.ajustar_pid(0.5, 0.001, 1) # <== AJUSTES PID
		self.controle_pouso.limitar_saida(0.1, 0.9)
		
		
		self.nave_atual.auto_pilot.engage() # LIGAR O PILOTO
		self.decolar()
		
		#Ativa freios e RCS para melhorar o controle da descida:
		self.nave_atual.control.brakes = True
		self.nave_atual.control.rcs = True
		self.controle_pouso.set_valor_limite(5)
		self.nav.navegacao(__class__.centro_espacial,self.nave_atual)
		while not (self.exec_suicide_burn): # LOOP esperando para executar o SuicideBurn
			self.atualizar_variaveis()
			self.nav.mirar_nave()
			print("Distância Até Queima: " , self.distancia_ate_queima)
			print("Altitude: " , self.alt_nave())
			print("Dist até Suicide: " , (self.alt_nave() - self.distancia_ate_queima - self.distancia_pouso))
				
			sleep(0.03)
			if ((0 >= self.alt_nave() - self.distancia_ate_queima - self.distancia_pouso) and (self.vel_vert_nave() < -1)): 
				self.exec_suicide_burn = True
			
		while (self.exec_suicide_burn):  # LOOP PRINCIPAL DE SUICIDE BURN
			print("Controle Acel PID: " , self.controle_acel.computar_pid())
			print("Distância Até Queima: " , self.distancia_ate_queima)
			print("Dist até Suicide: " , (self.alt_nave() - self.distancia_ate_queima - self.distancia_pouso))
			
			# Apontar nave para o retrograde se a velocidade horizontal for maior que 1m/s
			if (self.voo_nave.horizontal_speed < 0.01):
				self.nave_atual.control.target_pitch = 0
			else:
				self.nav.mirar_nave()
			
			# Descer pernas de pouso da nave em menos de 100 metros
			if (self.alt_nave() < 100):
				self.nave_atual.control.gear = True
				self.pode_pousar = True
	
			self.atualizar_variaveis() # atualiza valores
	
			# -=- Informa ao PID a altitude da nave e o limite -=-
# 			self.controle_acel.set_valor_entrada(self.alt_nave())
# 			self.controle_acel.set_valor_limite(self.distancia_pouso + self.distancia_ate_queima)
			self.controle_acel.set_valor_entrada(self.alt_nave())
			self.controle_acel.set_valor_limite(self.distancia_ate_queima + (self.distancia_pouso*0.50))
			self.controle_acel.limitar_saida(0.8/self.nave_twr_max, self.nave_twr_max*0.8)
		
			# -=- Corrigir a aceleração -=-
			if (((self.alt_nave()) < self.distancia_pouso)):
				if (self.vel_vert_nave() < 0):
					self.controle_pouso.set_valor_entrada(math.fabs(self.vel_vert_nave()))
				else:
					self.controle_pouso.set_valor_entrada(self.vel_vert_nave())
				try:
					self.nova_acel = math.fabs((1.5 - self.controle_pouso.computar_pid()) / self.nave_twr_max)
				except:
					self.nova_acel = 0
				self.nave_atual.control.throttle = self.nova_acel
			else:
				try:
					self.nova_acel = float((self.controle_acel.computar_pid()))
				except:
					self.nova_acel = 0
				self.nave_atual.control.throttle = (self.nova_acel)
			if (self.terminar_pouso()):
				self.nave_atual.auto_pilot.disengage() # DESLIGAR O PILOTO
				self.exec_suicide_burn = False
					
			sleep(0.03)
	
	def atualizar_variaveis(self): 
		#Calcular TWR:
		try:
			self.nave_twr_max = self.nave_atual.available_thrust / (self.massa_total_nave() * self.forca_grav)
		except:
			self.nave_twr_max = 1
		#Aceleração Máxima da Nave:
		self.acel_max = (self.nave_twr_max * self.forca_grav) - self.forca_grav
		#Tempo de queima dessa aceleração:
		try:
			self.tempo_da_queima = self.vel_vert_nave() / self.acel_max
		except:
			self.tempo_da_queima = 0
		#Distancia até iniciar a queima de suicide burn com essa aceleração:
		try:
			self.distancia_ate_queima = (self.vel_vert_nave() * self.tempo_da_queima + 1 / 2 * self.acel_max * math.pow(self.tempo_da_queima, 2))
		except:
			self.distancia_ate_queima = 0
		#Mostrar dados:
		try:
			self.txt_altitude.content = str("Altitude: " + "{0:.1f}".format((self.alt_nave()))+ "m")
		except:
			self.txt_altitude.content = "Impossível buscar altitude"
		try:
			self.txt_velocidade.content = str("Velocidade: " + "{0:.1f}".format((self.vel_vert_nave()))+ "m/s")
		except:
			self.txt_velocidade.content = "Impossível buscar velocidade"
		try:
			self.motores = self.nave_atual.parts.engines
			for motor in self.motores:
				self.empuxo_total += motor.max_thrust
			twr_atual = float(self.empuxo_total / (self.massa_total_nave() * self.forca_grav))
			self.txt_twr.content = str("TWR: " + "{0:.1f}".format((twr_atual)))
			self.empuxo_total = 0
		except:
			self.txt_twr.content = "Impossível buscar TWR"
		sleep(0.03)

	def decolar(self):
		#Se o veículo está pousado ou na base de lançamento, ele sobe:
		if (str(self.nave_atual.situation) == "VesselSituation.landed" 
			or str(self.nave_atual.situation) == "VesselSituation.pre_launch"):
			if str(self.nave_atual.situation) == "VesselSituation.pre_launch":
				self.nave_atual.control.activate_next_stage()
			self.nave_atual.control.gear = False
			self.atualizar_variaveis()
			try:
				self.nave_atual.control.throttle = (1.5 / self.nave_twr_max)
			except:
				self.nave_atual.control.throttle = 1
			while (self.alt_nave() <= self.distancia_pouso):
				self.nave_atual.control.target_pitch = 0
				sleep(0.03)
			self.nave_atual.control.throttle = 0
			sleep(0.03)
			self.nave_atual.control.target_pitch = 0 #mirar pra cima <<<
		else:	
			self.nave_atual.control.throttle = 0

	def terminar_pouso(self): 
		if ((str(self.nave_atual.situation) == "VesselSituation.landed")
				or (str(self.nave_atual.situation) == "VesselSituation.splashed") and (self.pode_pousar == True)):
			while (self.nave_atual.control.throttle > 0.1):
				acel = self.nave_atual.control.throttle - 0.3
				self.nave_atual.control.throttle = acel
				sleep(0.03)
			print("Pouso finalizado.")
			self.nave_atual.control.throttle = 0.0
			self.nave_atual.auto_pilot.disengage()
			self.nave_atual.control.brakes = False
			self.pode_pousar = False
			return True
		return False

	def construir_painel_info(self):
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
		self.txt_altitude.rect_transform.size = (400.0, __class__.margem_ui)
		self.txt_velocidade.rect_transform.size = (400.0, __class__.margem_ui)
		self.txt_twr.rect_transform.size = (400.0, __class__.margem_ui)
		self.txt_altitude.rect_transform.position = (__class__.margem_ui, __class__.margem_ui)
		self.txt_velocidade.rect_transform.position = (__class__.margem_ui, 0)
		self.txt_twr.rect_transform.position = (__class__.margem_ui, -__class__.margem_ui)
		self.txt_altitude.color = ( 0.0, 0.0, 0.0)
		self.txt_altitude.size = (16)
		self.txt_velocidade.color = ( 0.0, 0.0, 0.0)
		self.txt_velocidade.size = (16)
		self.txt_twr.color = ( 1.0, 1.0, 1.0)
		self.txt_twr.size = (16)
		
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
				self.txt_altitude.content = str("Altitude: " + "{0:.1f}".format((self.alt_nave()))+ "m")
			except:
				self.txt_altitude.content = "Impossível buscar altitude"
			try:
				self.txt_velocidade.content = str("Velocidade: " + "{0:.1f}".format((self.vel_vert_nave()))+ "m/s")
			except:
				self.txt_velocidade.content = "Impossível buscar velocidade"
			try:
				motores = self.nave_atual.parts.engines
				for motor in motores:
					self.empuxo_total += motor.max_thrust
				twr_atual = float(self.empuxo_total / (self.massa_total_nave() * self.forca_grav))
				self.txt_twr.content = str("TWR: " + "{0:.1f}".format((twr_atual)))
				self.empuxo_total = 0
			except:
				self.txt_twr.content = "Impossível buscar TWR"
			sleep(0.50)
			if bt_cancelar_clk():
				cancelar = True
				break
		if not cancelar:
			self.suicide_burn(conexao)
			bt_sburn.clicked = False
			bt_sburn.remove()
		
if (__name__ == "__main__"):
	conexao = krpc.connect(name="MechPeste")
	sb = SuicideBurn(conexao)
