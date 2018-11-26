from Vetor import Vetor
from ControlePID import ControlePID
import math

class Navegacao:
      
    def __init__(self):
        self.alinhar_distancia_horizontal = Vetor(0,0)
        self.posicao_pouso_alvo = (0.0, 0.0, 0.0)
        self.vetorDaVelocidade = Vetor(0, 0)
        self.controle_mag = ControlePID()
        self.alvo_ou_nao = False
        
    def navegacao(self, centro_espacial, nave_atual):
        self.centro_espacial = centro_espacial
        self.nave_atual = nave_atual
        self.ponto_ref = self.nave_atual.orbit.body.reference_frame
        self.voo_nave = self.nave_atual.flight(self.ponto_ref)
        self.controle_mag.set_tempo_amostra(25)
        self.controle_mag.ajustar_pid(0.5, 0.001, 10)
        self.controle_mag.limitar_saida(-1, 1)
        self.mirar_nave()
        
    def mirar_nave(self):
        self.posicao_pouso_alvo = self.centro_espacial.transform_position(self.voo_nave.retrograde, 
                                                                          self.nave_atual.surface_velocity_reference_frame, 
                                                                          self.ponto_ref)
        try:
            self.posicao_pouso_alvo = self.centro_espacial.target_vessel.position(self.ponto_ref) #ALVO
            self.alvo_ou_nao = True
        except:
            self.alvo_ou_nao = False
            
        self.alinhar_distancia_horizontal = Vetor.vetor_distancia(self,
            self.centro_espacial.transform_position(self.posicao_pouso_alvo, self.ponto_ref, self.nave_atual.surface_reference_frame),self.nave_atual.position(self.nave_atual.surface_reference_frame))
        
        self.alinhar_direcao = self.get_elevacao_direcao_do_vetor(self.alinhar_distancia_horizontal)
 
        self.nave_atual.auto_pilot.target_pitch_and_heading(self.alinhar_direcao.elevacao, self.alinhar_direcao.direcao)
        self.nave_atual.auto_pilot.target_roll = self.alinhar_direcao.direcao
 
 #       print(self.alinhar_direcao.toString())
 
#      # FIM DO METODO MAIN
# 
    def get_elevacao_direcao_do_vetor(self,vetor):
        self.vec = vetor
        self.to_ret = DirElev(0, 0)
        self.vec_invertido = Vetor(0,0)
        self.vel_relativa = self.centro_espacial.transform_position(self.voo_nave.velocity, self.ponto_ref, self.nave_atual.surface_reference_frame)
 
        self.vetorDaVelocidade.x = (self.vel_relativa[1])
        self.vetorDaVelocidade.y = (self.vel_relativa[2])
        self.vetorDaVelocidade = self.vetorDaVelocidade.inverte()
 
        self.vec = self.vec.subtrai(self.vetorDaVelocidade)
        if (self.alvo_ou_nao):
            self.vec_invertido = self.vec.multiplica(-1) #ALVO
        else:
            self.vec_invertido = self.vec.multiplica(1) #RETROCESSO
         
        self.to_ret.direcao = ((math.atan2(self.vec_invertido.x, self.vec_invertido.y) / math.pi) * 180)
        if (self.to_ret.direcao < 0):
            self.to_ret.direcao = 360 + self.to_ret.direcao
 
        self.comprimento = math.pow(1 + self.vec.magnitude(), 1) - 1
        self.controle_mag.set_valor_entrada(self.comprimento)
        self.controle_mag.set_valor_limite(self.vetorDaVelocidade.magnitude())
        self.to_ret.elevacao = max(60, (90 - self.comprimento * 2))
 
#         print("Comprimento: " + str(self.comprimento))
#         print("Comprimento Vel: " + str(self.vetorDaVelocidade.magnitude()))
#         print("Inclinacao: " + str(self.to_ret.elevacao))
#         print("PID: " + str(self.controle_mag.computar_pid()))
#  
        return self.to_ret


class DirElev(): 
    direcao = 0
    elevacao = 0
    
    def __init__(self, d, e):
    
        self.direcao = d
        self.elevacao = e
    
    def toString(self):
        return str(self.direcao) + str(self.elevacao)
    

