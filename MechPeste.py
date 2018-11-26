#!/usr/bin/python3

# Autor: Renan Torres <pesterenan@gmail.com>
# Data: 14/08/2018
# Mod MechPestePython

import DecolagemOrbital
import SuicideBurn
import krpc
import time
import traceback

"""MechPestePython mod de controle automático de foguetes"""

class MechPeste:
    def __init__(self):
        self.margem_ui = 20.0
        self.num_conexoes = 0
        self.conexao = None
        self.painel_info = None
        self.GRAV = 9.8175
        self.rodar()
        
        
    def rodar(self):
        try:
            print("Rodando...")
            self.iniciar_mechpeste()
            print("Aguardando Função.")
                # Loop principal:
            while True:
                if self.bt_suicide_clk():
                    self.bt_flutuar.clicked = False
                    self.bt_decolar.visible = False
                    self.bt_flutuar.visible = False
                    self.txt_painel_1.visible = True
                    self.txt_painel_2.visible = True
                    SuicideBurn.SuicideBurn(self.conexao)
                    
                                    
                # Se clicar em Decolar:
                if self.bt_decolar_clk():
                    self.bt_decolar.clicked = False
                    self.bt_decolar.visible = False
                    self.txt_painel_1.visible = True
                    self.txt_painel_2.visible = True
                    DecolagemOrbital.DecolagemOrbital(self.conexao)
                    break
                #Espera do Loop
                time.sleep(0.50)
            
        except Exception as e: 
            self.num_conexoes += 1
            print("Erro de conexão: {}".format(e))
            print(traceback.format_exc())       
            time.sleep(1)
            if (self.num_conexoes < 1):
                try:
                    if self.conexao != None:
                        self.conexao.close()
                        self.reset_painel()
                except Exception as e:
                    self.rodar()
                    
    def iniciar_mechpeste(self):
            # Conexão com o Server KRPC
        self.conexao = krpc.connect(name="MechPeste")
            # Buscar tela de jogo
        self.tela_itens = self.conexao.ui.stock_canvas
            # Tamanho da tela de jogo
        self.tamanho_tela = self.tela_itens.rect_transform.size
            # Adicionar Painel com os botões
        self.painel_bts = self.adicionar_painel(self.tela_itens)
            # Streams para ver quando os botões são clicados:
        self.bt_pouso_clk = self.conexao.add_stream(getattr, self.bt_pousar, "clicked")
        self.bt_suicide_clk = self.conexao.add_stream(getattr, self.bt_flutuar, "clicked")
        self.bt_decolar_clk = self.conexao.add_stream(getattr, self.bt_decolar, "clicked")
            
    def adicionar_painel(self,tela):
            # Adicionar Painel usando o canvas como argumento
        self.painel_info = tela.add_panel()
            # Posicionar o painel e movê-lo à esquerda da tela
        self.retangulo = self.painel_info.rect_transform
        self.retangulo.size = (450, 40)
        self.retangulo.position = (0, (self.tamanho_tela[1]) / 2 - 85)
            # Adicionar Botões ao painel do Mod:
        self.bt_decolar = self.painel_info.add_button("Decolar")
        self.bt_decolar.rect_transform.size = (125, 30)
        self.bt_decolar.rect_transform.position = (-150, 0)
    
        self.bt_flutuar = self.painel_info.add_button("Flutuar")
        self.bt_flutuar.rect_transform.size = (125, 30)
        self.bt_flutuar.rect_transform.position = (0, 0)
    
        self.bt_pousar = self.painel_info.add_button("Pousar")
        self.bt_pousar.rect_transform.size = (125, 30)
        self.bt_pousar.rect_transform.position = (150, 0)
            # Adicionar texto mostrando informações:
        self.txt_painel_1 = self.painel_info.add_text("Alt:", False)
        self.txt_painel_1.rect_transform.position = (10, 0)
        self.txt_painel_1.rect_transform.size = (self.retangulo.size[0], self.margem_ui)
        self.txt_painel_1.color = (1, 1, 1)
        self.txt_painel_1.size = 18
    
        self.txt_painel_2 = self.painel_info.add_text("0", False)
        self.txt_painel_2.rect_transform.position = (50, 0)
        self.txt_painel_2.rect_transform.size = (self.retangulo.size[0], self.margem_ui)
        self.txt_painel_2.color = (1,1,1)
        self.txt_painel_2.size = 18
    
            #Retorna o objeto de painel
        return self.painel_info
    
    
    def set_txt_painel_1(self,texto):
        self.txt_painel_1.content(texto)
    
    def set_txt_painel_2(self,texto):
        self.txt_painel_2.content(texto)
    
    def reset_painel(self):
        self.bt_flutuar.clicked = False
        self.bt_decolar.clicked = False
        self.bt_pousar.clicked = False
            
        self.bt_pousar.visible = True
        self.bt_decolar.visible = True
        self.bt_flutuar.visible = True
                    
        self.txt_painel_1.visible = False
        self.txt_painel_2.visible = False
        self.txt_painel_1.content = ""
        self.txt_painel_2.content = ""
        
if (__name__ == "__main__"):
        
    mp = MechPeste()
