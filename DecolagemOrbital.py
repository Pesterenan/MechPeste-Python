import krpc
import math
from ControlePID import ControlePID
from time import sleep

"""MechPestePython - Módulo de decolagem"""

class DecolagemOrbital():
	#Elementos da UI:
	margem_ui = 20
	#Parametros:
	GRAV = 9.81
	conexao = None
	centro_espacial = None
		
	def __init__(self, conexao):
		__class__.conexao = conexao
		__class__.centro_espacial = conexao.space_center
		self.iniciar_decolagem()
		
	def iniciar_decolagem(self):
		#Parametros padrao de decolagem:
		self.alt_inicio_curva = 150
		self.alt_fim_curva = 60000
		self.alt_final = 85000
		self.inclinacao = "Normal"
		self.programa = 0
		self.escudos_soltos = False
		#Parametros de voo:
		self.nave_atual = __class__.centro_espacial.active_vessel # objeto da nave
		ponto_ref = self.nave_atual.orbit.body.reference_frame
		voo_nave = self.nave_atual.flight(ponto_ref)
		self.ut = __class__.conexao.add_stream(getattr, __class__.centro_espacial, "ut" )
		self.controle_acel = ControlePID()
				
		#Variaveis da nave:
		self.altitude = __class__.conexao.add_stream(getattr, voo_nave, "mean_altitude" )
		self.max_q = __class__.conexao.add_stream(getattr, voo_nave, "dynamic_pressure" )
		self.apoastro = __class__.conexao.add_stream(getattr, self.nave_atual.orbit, "apoapsis_altitude" )
		self.periastro = __class__.conexao.add_stream(getattr, self.nave_atual.orbit, "periapsis_altitude" )
		self.tempo_ate_apoastro = __class__.conexao.add_stream(getattr, self.nave_atual.orbit, "time_to_apoapsis")
		
		self.construir_painel_info() #<<< Mostrar painel de informacoes

		while True:
			if self.programa == 0:
				self.iniciar_lancamento()
			elif self.programa == 1:
				self.giro_gravitacional()
			elif self.programa == 2:
				self.circularizar()

			sleep(0.40)

	def iniciar_lancamento(self):
		#	Iniciar Lancamento:
		self.nave_atual.control.sas = False # desligar SAS
		self.nave_atual.control.rcs = False # desligar RCS
			
		#	Contagem regressiva...
		self.set_txt_titulo("Lançamento em:")
		for segundo in range(5,0,-1):
			self.set_txt_status(segundo)
			sleep(1)
			
		#	Acertar aceleracao:
		motores = self.nave_atual.parts.engines
		empuxo_total_lancamento = motores[0].max_thrust
		massa_total_lancamento = self.nave_atual.mass #	massa da nave
		try:
			acel_lancamento = float((1.5 / (empuxo_total_lancamento / (massa_total_lancamento * __class__.GRAV))))
		except:
			acel_lancamento = 1
		self.nave_atual.control.throttle = acel_lancamento # ACELERAR COM 1.5 DE TWR
	
		#	Lançar e ativar piloto automático
		if (str(self.nave_atual.situation) == "VesselSituation.pre_launch"):
			self.nave_atual.control.activate_next_stage()
		self.nave_atual.auto_pilot.engage() # ativa o piloto auto
		self.nave_atual.auto_pilot.target_pitch_and_heading(90, 90) # direção
		self.programa = 1

	def giro_gravitacional(self):
		self.set_txt_titulo("Altitude em Relação ao Solo:") 
		
		#Controle de Aceleração
		self.controle_acel.set_valor_limite(25000) #Limite de pressão dinâmica
		self.controle_acel.limitar_saida(0.9 , 3) #Limite de controle do TWR
		self.controle_acel.ajustar_pid(0.001,0.001,0.0001)
		angulo_giro = 0 # angulo de giro
		
		while (True):  # loop do giro de gravidade
			self.set_txt_status(("{0:.1f}".format(self.altitude()) + " metros")) #Mostra Altitude
			# Giro de Gravidade
			if (self.altitude() > float(self.alt_inicio_curva) and self.altitude() < float(self.alt_fim_curva)):
				try:
					incremento = math.sqrt((self.altitude() - self.alt_inicio_curva) / (self.alt_fim_curva - self.alt_inicio_curva))
				except:
					pass

				if self.inclinacao == "Rasa":
					novo_angulo = math.sqrt(incremento* 0.8) * 90.0
				elif self.inclinacao == "Normal":
					novo_angulo = (incremento) * 85.0
				elif self.inclinacao == "Aguda":
					novo_angulo = math.pow(incremento,1.2) * 80.0
				
				if (math.fabs(novo_angulo - angulo_giro) > 0.3):
					angulo_giro = novo_angulo
					self.nave_atual.auto_pilot.target_pitch_and_heading(float(90 - angulo_giro), 90)
				
			#Controlar Aceleração para evitar o MaxQ:		
			self.controle_acel.set_valor_entrada(self.max_q())
			try:
				nova_acel = float((self.controle_acel.computar_pid() / (self.nave_atual.available_thrust / (self.nave_atual.mass * __class__.GRAV))))
			except:
				nova_acel = 0
			self.nave_atual.control.throttle = nova_acel
			
			if (self.altitude() > 20000):
				self.controle_acel.set_valor_limite(20000)
			
			# Diminuir aceleracao ao chegar perto do apoastro alvo
			if (self.apoastro() >= self.alt_final * 0.95):
				self.set_txt_titulo("Aproximando-se do apoastro alvo")
				self.nave_atual.control.throttle = 0.20 # mudar aceleração pra 25%
				break

			self.soltar_escudos()
			sleep(0.40)
			
		# Desativa motores ao chegar no apoastro
		while (self.apoastro() < self.alt_final):
			self.set_txt_titulo("Altitude do Apoastro:") 
			self.set_txt_status("{0:.1f}".format(self.apoastro()) + "m, alvo: "+ "{0:.1f}".format(self.alt_final) +"m.")
			self.soltar_escudos()
			sleep(0.40)
		self.set_txt_titulo("Apoastro alvo alcançado") 
		self.nave_atual.control.throttle = 0 # cortar motor
		sleep(1)
		# esperar ate sair da atmosfera
		self.set_txt_titulo("Esperando sair da atmosfera")
		while (self.altitude() < 65000): 
			self.set_txt_status("Altitude: " + "{0:.1f}".format(self.altitude()) + " metros")
			self.soltar_escudos()
			sleep(0.40)
		self.programa = 2

	def soltar_escudos(self):
		#	Soltar Fairings da nave:
		if (self.altitude() > 50000 and self.escudos_soltos == False):
			for escudo in self.nave_atual.parts.fairings:
				try:
					if not (escudo.jettisoned):
						self.set_txt_status("Soltando escudos...")
						sleep(1)
						escudo.jettison()
						self.set_txt_status("Escudos soltos.")
						self.escudos_soltos = True
						sleep(1)
				except:
					self.set_txt_status("Erro ao soltar escudos...")
					sleep(1)						
	
	def circularizar(self):
		# Planejar circularizacao
		self.set_txt_titulo("Planejando queima de circularização") 
		gm = self.nave_atual.orbit.body.gravitational_parameter # pegar parametro G
		apo = self.nave_atual.orbit.apoapsis # apoastro da orbita
		sma = self.nave_atual.orbit.semi_major_axis # semieixo da orbita
		v1 = math.sqrt(gm * ((2.0 / apo) - (1.0 / sma)))
		v2 = math.sqrt(gm * ((2.0 / apo) - (1.0 / apo)))
		delta_v = v2 - v1

		#Adicionar a manobra
		node = self.nave_atual.control.add_node(self.ut() + self.nave_atual.orbit.time_to_apoapsis, prograde = delta_v)
		
		# Calcular tempo de queima (equacao de foguete)
		massa_total = self.nave_atual.mass # massa total do foguete
		isp = self.nave_atual.specific_impulse * __class__.GRAV # isp multiplicado pela constante grav
		empuxo_total = self.nave_atual.available_thrust # empuxo disponivel
		massa_seca = massa_total / math.exp(delta_v / isp) # massa seca do foguete
		taxa_queima = empuxo_total / isp # taxa de fluxo, empuxo / isp
		tempo_queima = (massa_total - massa_seca) / taxa_queima #tempo de queima

		# Orientar nave ao noh
		self.set_txt_status("Orientando nave para queima de circularização") 
		self.nave_atual.control.rcs = True
		self.nave_atual.auto_pilot.reference_frame = node.reference_frame
		self.nave_atual.auto_pilot.target_direction = (0.0, 1.0, 0.0)
		
		# esperar ate a queima
		self.set_txt_titulo("Esperando até a queima de circularização") 
		tempo_ate_queima = float(self.ut() + self.nave_atual.orbit.time_to_apoapsis - (tempo_queima / 2.0))
		self.set_txt_status(("Tempo de queima: " + "{0:.1f}".format(tempo_queima) + " segundos"))
		espera = 5
		self.nave_atual.auto_pilot.wait()
		__class__.centro_espacial.warp_to(tempo_ate_queima - espera)
		self.nave_atual.auto_pilot.target_direction = (0.0, 1.0, 0.0)
		
		# executar queima
		self.set_txt_titulo("Pronto para executar queima") 
		queima_restante = __class__.conexao.add_stream(node.remaining_burn_vector, node.reference_frame)
		while (self.tempo_ate_apoastro() - (tempo_queima / 2.0) > 0.1) :
			self.set_txt_status(str("Ignição em " + "{0:.1f}".format(self.tempo_ate_apoastro() - (tempo_queima / 2.0)) + " segundos..."))
			
		self.set_txt_titulo("Executando queima") 
		self.nave_atual.control.throttle = 1
		while (queima_restante()[1] > 1):
			sleep(0.20)
			if (queima_restante()[1] < 50 and self.nave_atual.control.throttle > 0.1):
				self.set_txt_titulo("Ajustando...") 
				self.nave_atual.control.throttle = 0.1
			self.set_txt_status("Delta V restante: " + "{0:.1f}".format(queima_restante()[1]) + "m/s")
		self.nave_atual.control.throttle = 0
		node.remove()
		self.set_txt_titulo("Lançamento completo.") 
		self.set_txt_status("Apoastro: "+"{0:.1f}".format(self.apoastro())+"m, Periastro: "+"{0:.1f}".format(self.periastro())+"m")
		self.nave_atual.auto_pilot.disengage()
		self.nave_atual.control.sas = True
		self.incremento = 0
		self.angulo_giro = 0
		self.novo_angulo = 0
		sleep(5)
		__class__.conexao.close()

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
	
		
		# Adicionar caixa de texto para altitude
		cx_txt = painel_info.add_input_field(True)
		self.txt_titulo = painel_info.add_text("Digite a Altitude Final: ", True)
		self.txt_status = painel_info.add_text("", False)
	
		self.txt_titulo.rect_transform.size = (400.0, __class__.margem_ui)
		self.txt_status.rect_transform.size = (400.0, __class__.margem_ui)
	
		self.txt_titulo.rect_transform.position = (__class__.margem_ui, __class__.margem_ui)
		self.txt_status.rect_transform.position = (__class__.margem_ui, -__class__.margem_ui)
	
		self.txt_titulo.color = ( 0.0, 0.0, 0.0)
		self.txt_titulo.size = (14)
	
		self.txt_status.color = ( 1.0, 1.0, 1.0)
		self.txt_status.size = (18)
		cx_txt.rect_transform.position = (-100.0, -__class__.margem_ui)
		cx_txt.value = str(self.alt_final)

		# Adicionar botões:
		bt_decolar = painel_info.add_button("Decolar", True)
		bt_configurar = painel_info.add_button("Configurar", True)
		bt_ok = painel_info.add_button("OK", False)
		bt_inc_nor = painel_info.add_button("Normal", False)
		bt_inc_agu = painel_info.add_button("Aguda", False)
		bt_inc_ras = painel_info.add_button("Rasa", False)
		bt_decolar.rect_transform.position = (100.0, -__class__.margem_ui)
		bt_configurar.rect_transform.position = (100.0, __class__.margem_ui)
		bt_ok.rect_transform.position = (100.0, -__class__.margem_ui)
		bt_inc_nor.rect_transform.position = (-100.0, -__class__.margem_ui)
		bt_inc_agu.rect_transform.position = (-100.0, -__class__.margem_ui)
		bt_inc_ras.rect_transform.position = (-100.0, -__class__.margem_ui)


		# stream para checar se o botao foi clicado
		bt_decolar_clk = __class__.conexao.add_stream(getattr, bt_decolar, "clicked")
		bt_ok_clk = __class__.conexao.add_stream(getattr, bt_ok, "clicked")
		bt_configurar_clk = __class__.conexao.add_stream(getattr, bt_configurar, "clicked")
		bt_inc_nor_clk = __class__.conexao.add_stream(getattr, bt_inc_nor, "clicked")
		bt_inc_agu_clk = __class__.conexao.add_stream(getattr, bt_inc_agu, "clicked")
		bt_inc_ras_clk = __class__.conexao.add_stream(getattr, bt_inc_ras, "clicked")

		# Esperar clique do botão:
		while not (bt_decolar_clk()):
			numero = True
			if (cx_txt.changed):
				try:
					self.alt_final = float(cx_txt.value)
				except Exception: 
					numero = False
				
				if (numero) and (self.alt_final > 70500.0):
					self.set_txt_titulo("Digite a Altitude Final: ") 
					bt_decolar.visible = True
					bt_configurar.visible = True
				else:
					self.set_txt_titulo("Precisa ser um número, acima de 71km!") 
					bt_decolar.visible = False
					bt_configurar.visible = False
			sleep(0.50)
			if bt_configurar_clk():
				self.set_txt_titulo("Escolha a inclinação de subida orbital:")
				bt_inc_nor.visible = True
				bt_ok.visible = True
				bt_configurar.visible = False
				bt_decolar.visible = False
				cx_txt.visible = False
			
				while (True):
					if bt_inc_nor_clk():
						self.inclinacao = "Aguda"
						bt_inc_nor.clicked = False
						bt_inc_nor.visible = False
						bt_inc_agu.visible = True
								
					if bt_inc_agu_clk():
						self.inclinacao = "Rasa"
						bt_inc_agu.clicked = False
						bt_inc_agu.visible = False
						bt_inc_ras.visible = True
					if bt_inc_ras_clk():
						self.inclinacao = "Normal"
						bt_inc_ras.clicked = False
						bt_inc_ras.visible = False
						bt_inc_nor.visible = True		
						
					if bt_ok_clk():
						bt_decolar.visible = True
						bt_configurar.visible = True
						bt_ok.visible = False
						bt_inc_nor.visible = False
						bt_inc_agu.visible = False
						bt_inc_ras.visible = False
						bt_ok.clicked = False
						bt_configurar.clicked = False
						cx_txt.visible = True
						self.set_txt_titulo("Digite a Altitude Final: ") 
						break
								
		bt_inc_ras.remove()
		bt_inc_nor.remove()
		bt_inc_agu.remove()
		bt_ok.remove()
		bt_decolar.remove()
		bt_configurar.remove()
		cx_txt.remove()
		self.txt_status.visible = True
	
	def set_txt_titulo(self, texto):
		self.txt_titulo.content = texto
		
	def set_txt_status(self, texto):
		self.txt_status.content = ("Status: " + str(texto))

if (__name__ == "__main__"):
	conexao = krpc.connect(name="MechPeste")
	do = DecolagemOrbital(conexao)
