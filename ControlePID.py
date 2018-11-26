#!/usr/bin/python3

# Autor: Renan Torres <pesterenan@gmail.com>
# Data: 22/08/2018
# Controlador PID

import math
import time

class ControlePID:
	"""Controlador PID escrito em Python para uso com o mod MechPeste"""
	__name__ = "ControlePID"
	saida_min, saida_max = 0, 1
	kp, ki, kd = 0.025,0.001,0.95 # variaveis de ajuste do PID
	tempo_amostra = 0.02 # tempo de amostragem
	
	def __init__(self):
		super(ControlePID, self).__init__()
		self.valor_entrada, self.valor_saida, self.valor_limite = 0, 0, 0
		self.ult_valor_entrada = 0 # variáveis de cálculo de erro
		self.ult_calculo = 0 #tempo do último cálculo
		self.termo_integral = 0
	
	#-=- Código principal, para computar o PID -=-
	def computar_pid(self): 
		self.agora = time.time() # variável que busca o tempo imediato
		self.mudanca_tempo = self.agora - self.ult_calculo # variável que compara o tempo de cálculo
		
		if (self.mudanca_tempo >= self.tempo_amostra): #se a mudança for maior que o tempo de amostra, o cálculo é feito.
		#variáveis para o cálculo do valor de saída
			self.erro = self.valor_limite - self.valor_entrada
			self.termo_integral += self.ki * self.erro
			if (self.termo_integral > self.saida_max): 
				self.termo_integral = self.saida_max
			elif (self.termo_integral < self.saida_min): 
				self.termo_integral = self.saida_min
			self.dvalor_entrada = (self.valor_entrada - self.ult_valor_entrada)
				
			#computando o valor de saída
			self.valor_saida = self.kp * self.erro + self.ki * self.termo_integral - self.kd * self.dvalor_entrada
			#-=-Limitando o valor de saída-=-
			if (self.valor_saida > self.saida_max): 
				self.valor_saida = self.saida_max
			elif (self.valor_saida < self.saida_min): 
				self.valor_saida = self.saida_min
				
		#-=-Relembrando os valores atuais para a próxima vez
		self.ult_valor_entrada = self.valor_entrada
		self.ult_calculo = self.agora
			
		#-=-Retornando o valor para quem chamou esse método
		return self.valor_saida
		

	def set_valor_entrada(self,valor): 
		if (valor > 0): 
			self.valor_entrada = valor
		else:
			self.valor_entrada = 0
			
		
	def set_valor_limite(self,valor): 
		if (valor > 0): 
			self.valor_limite = valor
		else:
			self.valor_limite = 0
		
	def limitar_saida(self,minimo, maximo): 
		if (minimo > maximo): 
			return
		self.saida_min = minimo
		self.saida_max = maximo

		if (self.termo_integral > self.saida_max): 
			self.termo_integral = self.saida_max
		elif (self.termo_integral < self.saida_min): 
			self.termo_integral = self.saida_min

		if (self.valor_saida > self.saida_max): 
			self.valor_saida = self.saida_max
		elif (self.valor_saida < self.saida_min): 
			self.valor_saida = self.saida_min
		
	def ajustar_pid(self, Kp,  Ki,  Kd): 
		self.kp = Kp
		self.ki = Ki
		self.kd = Kd
		
	def set_tempo_amostra(self, tempo_amostra): 
		if (tempo_amostra > 0): 
			self.tempo_amostra = tempo_amostra/1000
			


