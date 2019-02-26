import Vetor
from ControlePID import ControlePID
import math


class Navegacao:
      
    def __init__(self):
        self.alinhar_distancia_horizontal = Vetor.Vetor(0, 0)
        self.posicao_pouso_alvo = Vetor.Vetor(0.0, 0.0)
        self.vetorDaVelocidade = Vetor.Vetor(0, 0)
        self.controle_mag = ControlePID()
        self.retornar_vetor = Vetor.Vetor(0, 0)

    def navegacao(self, centro_espacial, nave_atual):
        self.centro_espacial = centro_espacial
        self.nave_atual = nave_atual
        self.ponto_ref = self.nave_atual.orbit.body.reference_frame
        self.voo_nave = self.nave_atual.flight(self.ponto_ref)
        self.controle_mag.tempo_amostragem(40)
        self.controle_mag.ajustar_pid(0.5, 0.001, 10)
        self.controle_mag.limitar_saida(-1, 1)
        self.mirar_nave()
        
    def mirar_nave(self):
        # Buscar Nó Retrógrado:
        self.posicao_pouso_alvo = \
            self.centro_espacial.transform_position(self.voo_nave.retrograde,
                                                    self.nave_atual.surface_velocity_reference_frame,
                                                    self.ponto_ref)
        self.alinhar_distancia_horizontal = \
            Vetor.vetor_distancia(self.centro_espacial.transform_position(self.posicao_pouso_alvo,
                                                                          self.ponto_ref,
                                                                          self.nave_atual.surface_reference_frame),
                                  self.nave_atual.position(self.nave_atual.surface_reference_frame))
        
        self.alinhar_direcao = self.get_elevacao_direcao_do_vetor(self.alinhar_distancia_horizontal)
 
        self.nave_atual.auto_pilot.target_pitch_and_heading(self.alinhar_direcao.y, self.alinhar_direcao.x)
        self.nave_atual.auto_pilot.target_roll = self.alinhar_direcao.x

    def get_elevacao_direcao_do_vetor(self, vetor):
        self.vel_relativa = self.centro_espacial.transform_position(self.voo_nave.velocity,
                                                                    self.ponto_ref,
                                                                    self.nave_atual.surface_reference_frame)
 
        self.vetorDaVelocidade.x = (self.vel_relativa[1])
        self.vetorDaVelocidade.y = (self.vel_relativa[2])

        vetor = vetor.subtrai(self.vetorDaVelocidade)

        self.retornar_vetor.x = Vetor.angulo_direcao(vetor)

        self.comprimento = math.pow(1 + vetor.magnitude(), 1) - 1
        self.controle_mag.entrada_pid(self.comprimento)
        self.controle_mag.limite_pid(self.vetorDaVelocidade.magnitude())
        self.retornar_vetor.y = max(60, (90 - int(self.comprimento) * 2))
 
#         print("Comprimento: " + str(self.comprimento))
#         print("Comprimento Vel: " + str(self.vetorDaVelocidade.magnitude()))
#         print("Inclinacao: " + str(self.retornar_vetor.elevacao))
#         print("PID: " + str(self.controle_mag.computar_pid()))
#  
        return self.retornar_vetor
