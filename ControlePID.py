#!/usr/bin/env python3

# Autor: Renan Torres <pesterenan@gmail.com>
# Data: 22/08/2018 
# Atualizado: 10/01/2019
# Controlador Proporcional Integral Derivativo

import time


class ControlePID:
	"""Controlador PID escrito em Python para uso com o mod MechPeste"""
	__name__ = "Controlador PID"
	saida_min, saida_max = -1, 1 	# valores de saída padrão
	kp, ki, kd = 0.025, 0.001, 0.95 	# variáveis de ajuste padrão do PID
	tempo_amostra = 0.02 			# tempo de amostragem padrão
	
	def __init__(self):
		super(ControlePID, self).__init__()
		self.valor_entrada, self.valor_saida, self.valor_limite = 0, 0, 0
		self.ult_valor_entrada = 0 	# variáveis de cálculo de erro
		self.ult_calculo = 0 		# tempo do último cálculo
		self.termo_integral = 0
		self.agora = 0
		self.mudanca_tempo = 0
		self.erro = 0
		self.dvalor_entrada = 0

	def computar_pid(self): 
		"""Código principal, para computar o PID"""
		self.agora = float(time.time())
		# variável que busca o tempo imediato
		self.mudanca_tempo = float(self.agora - self.ult_calculo)
		# variável que compara o tempo de cálculo
		if self.mudanca_tempo >= self.tempo_amostra:
			# se a mudança for maior que o tempo de amostra, o cálculo é feito.
			# variáveis para o cálculo do valor de saída
			self.erro = self.valor_limite - self.valor_entrada
			self.termo_integral += self.ki * self.erro
			if self.termo_integral > self.saida_max:
				self.termo_integral = self.saida_max
			elif self.termo_integral < self.saida_min:
				self.termo_integral = self.saida_min
			self.dvalor_entrada = (self.valor_entrada - self.ult_valor_entrada)
				
			# computando o valor de saída
			self.valor_saida = self.kp * self.erro + self.ki * self.termo_integral - self.kd * self.dvalor_entrada
			# Limitando o valor de saída
			if self.valor_saida > self.saida_max:
				self.valor_saida = self.saida_max
			elif self.valor_saida < self.saida_min:
				self.valor_saida = self.saida_min
				
		# Relembrando os valores atuais para a próxima vez
		self.ult_valor_entrada = self.valor_entrada
		self.ult_calculo = self.agora
		
		# Retornando o valor para quem chamou esse método
		return self.valor_saida
		
	def entrada_pid(self, valor=None):
		"""Valor de entrada (comparação) do cálculo PID"""
		if valor is None:
			return self.valor_entrada
		self.valor_entrada = valor
				
	def limite_pid(self, valor=None):
		"""Valor limite, para comparar o valor de entrada do cálculo PID"""
		if valor is None:
			return self.valor_limite
		self.valor_limite = valor
			
	def limitar_saida(self, minimo, maximo):
		"""Restringe o valor de saída do cálculo PID"""
		if minimo > maximo:
			return
		self.saida_min = minimo
		self.saida_max = maximo

		if self.termo_integral > self.saida_max:
			self.termo_integral = self.saida_max
		elif self.termo_integral < self.saida_min:
			self.termo_integral = self.saida_min

		if self.valor_saida > self.saida_max:
			self.valor_saida = self.saida_max
		elif self.valor_saida < self.saida_min:
			self.valor_saida = self.saida_min
		
	def ajustar_pid(self, kp=None,  ki=None,  kd=None): 
		"""Ajuste dos parâmetros do cálculo PID
		Recebe valores novos se válidos, e retorna os valores"""
		if kp is None:
			return self.kp
		if ki is None:
			return self.ki
		if kd is None:
			return self.kd
		self.kp = kp if kp > 0 else __class__.kp
		self.ki = ki if ki > 0 else __class__.ki
		self.kd = kd if kd > 0 else __class__.kd
		
	def tempo_amostragem(self, tempo_amostra=None):
		"""Tempo para fazer amostragem dos valores em milissegundos"""
		if tempo_amostra is None:
			return self.tempo_amostra
		self.tempo_amostra = float(tempo_amostra/1000) if tempo_amostra > 0 else __class__.tempo_amostra
