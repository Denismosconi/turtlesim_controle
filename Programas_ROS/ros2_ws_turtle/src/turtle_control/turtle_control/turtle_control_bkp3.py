#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np
import time
import math
import json
import csv
import os
from datetime import datetime

###########################################################################################
#CRIANDO O NÓ DE CONTROLE
###########################################################################################
class ControlTurtle(Node):
    def __init__(self, target_x, target_y, Kp_angular, Kp_linear, experiment_name=None):
        super().__init__('turtle_control')
        self.posicao_sensor = self.create_subscription(Pose, "turtle1/pose", self.callback_controle, 10)
        self.velocidade_atuador = self.create_publisher(Twist, "turtle1/cmd_vel", 10)

###########################################################################################
#PREPRAÇÃO DE VARIÁVEIS E AMBIENTE
###########################################################################################

        # Configuração do experimento
        self.experiment_name = experiment_name or f"exp_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.save_dir = "turtle_experiments"
        
        # Criar diretório para salvar dados, se não existir
        os.makedirs(self.save_dir, exist_ok=True)

        # Variáveis para armazenamento dos dados
        self.x_positions = []
        self.y_positions = []
        self.theta_rads = []
        self.theta_degs = []
        self.timestamps = []
        self.estados = []  # Para registrar o estado em cada amostra
        
        #VETORES PARA DADOS DE CONTROLE
        self.theta_error_x_list = []
        self.theta_error_y_list = []
        self.velocidade_angular_x_list = []
        self.velocidade_angular_y_list = []
        self.velocidade_x_list = []
        self.velocidade_y_list = []
        self.posicao_error_x_list = []
        self.posicao_error_y_list = []

        #VETORES SEPARADOS POR FASE
        # Fase ROTACIONAR_X
        self.x_positions_rot_x = []
        self.y_positions_rot_x = []
        self.theta_rads_rot_x = []
        self.theta_degs_rot_x = []
        self.timestamps_rot_x = []
        
        # Fase MOVER_X
        self.x_positions_move_x = []
        self.y_positions_move_x = []
        self.theta_rads_move_x = []
        self.theta_degs_move_x = []
        self.timestamps_move_x = []
        
        # Fase ROTACIONAR_Y
        self.x_positions_rot_y = []
        self.y_positions_rot_y = []
        self.theta_rads_rot_y = []
        self.theta_degs_rot_y = []
        self.timestamps_rot_y = []
        
        # Fase MOVER_Y
        self.x_positions_move_y = []
        self.y_positions_move_y = []
        self.theta_rads_move_y = []
        self.theta_degs_move_y = []
        self.timestamps_move_y = []

        # Variáveis de tempo
        self.start_time = time.time()
        self.last_sample_time = self.start_time
        self.sample_interval = 0.001  # tempo de amostragem

        # Variáveis para controle de posição
        self.target_x = target_x
        self.target_y = target_y

        #Ganho do controlador proporcional
        self.Kp_angular = Kp_angular
        self.Kp_linear = Kp_linear

        #Timer para encerrar o programa
        self.shutdown_timer = None
        self.program_ending = False
        self.should_shutdown = False  # Flag para controle de shutdown

        # Executor para gerenciar o shutdown
        self.executor = None

        # Máquina de estados do controlador
        self.estado = "ROTACIONAR_X"  # Estados: ROTACIONAR_X, MOVER_X, ROTACIONAR_Y, MOVER_Y, FINALIZADO
        self.angle_tolerance = 0.001  # tolerância para posição angular
        self.position_tolerance = 0.001  # tolerância para posição linear

        # Controle de memória para evitar oscilações
        self.angle_corrected = False
        self.x_reached = False

        # Variáveis temporárias para cálculo
        self.theta_d_rad_x = 0.0
        self.theta_d_rad_y = 0.0
        self.theta_error_x = 0.0
        self.theta_error_y = 0.0
        self.posicao_error_x = 0.0
        self.posicao_error_y = 0.0
        self.velocidade_angular_x = 0.0
        self.velocidade_angular_y = 0.0
        self.velocidade_x = 0.0
        self.velocidade_y = 0.0

###########################################################################################
#MÉTODOS DE SALVAMENTO DE DADOS
###########################################################################################

    def save_all_data(self):
        """Salva todos os dados em JSON e CSV"""
        self.get_logger().info(f"Salvando dados do experimento: {self.experiment_name}")
        
        # Salvar em JSON
        self.save_to_json()
        
        # Salvar em CSV
        self.save_to_csv()
        
        # Salvar dados por fase
        self.save_phase_data()
        
        self.get_logger().info("Dados salvos com sucesso!")
    
    def save_to_json(self):
        """Salva todos os dados em formato JSON"""
        data_dict = {
            'metadata': {
                'experiment_name': self.experiment_name,
                'timestamp': datetime.now().isoformat(),
                'target_position': {'x': self.target_x, 'y': self.target_y},
                'controller_gains': {'Kp_angular': self.Kp_angular, 'Kp_linear': self.Kp_linear},
                'tolerances': {'angle': self.angle_tolerance, 'position': self.position_tolerance},
                'sample_interval': self.sample_interval,
                'total_samples': len(self.timestamps),
                'total_time': self.timestamps[-1] if self.timestamps else 0
            },
            'data': {
                'time': self.timestamps,
                'x': self.x_positions,
                'y': self.y_positions,
                'theta_rad': self.theta_rads,
                'theta_deg': self.theta_degs,
                'state': self.estados,
                'theta_error_x': self.theta_error_x_list,
                'theta_error_y': self.theta_error_y_list,
                'angular_vel_x': self.velocidade_angular_x_list,
                'angular_vel_y': self.velocidade_angular_y_list,
                'linear_vel_x': self.velocidade_x_list,
                'linear_vel_y': self.velocidade_y_list,
                'position_error_x': self.posicao_error_x_list,
                'position_error_y': self.posicao_error_y_list
            },
            'phase_summary': {
                'ROTACIONAR_X': len(self.timestamps_rot_x),
                'MOVER_X': len(self.timestamps_move_x),
                'ROTACIONAR_Y': len(self.timestamps_rot_y),
                'MOVER_Y': len(self.timestamps_move_y)
            }
        }
        
        filename = os.path.join(self.save_dir, f"{self.experiment_name}.json")
        with open(filename, 'w') as f:
            json.dump(data_dict, f, indent=2, default=self.json_serializer)
    
    def save_to_csv(self):
        """Salva dados principais em formato CSV (MATLAB-friendly)"""
        # Dados principais (uma grande tabela)
        filename = os.path.join(self.save_dir, f"{self.experiment_name}_main.csv")
        
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Cabeçalho
            writer.writerow([
                'time', 'x', 'y', 'theta_rad', 'theta_deg', 'state',
                'theta_error_x', 'theta_error_y',
                'angular_vel_x', 'angular_vel_y',
                'linear_vel_x', 'linear_vel_y',
                'position_error_x', 'position_error_y'
            ])
            
            # Dados
            for i in range(len(self.timestamps)):
                writer.writerow([
                    self.timestamps[i],
                    self.x_positions[i],
                    self.y_positions[i],
                    self.theta_rads[i],
                    self.theta_degs[i],
                    self.estados[i],
                    self.theta_error_x_list[i] if i < len(self.theta_error_x_list) else 0,
                    self.theta_error_y_list[i] if i < len(self.theta_error_y_list) else 0,
                    self.velocidade_angular_x_list[i] if i < len(self.velocidade_angular_x_list) else 0,
                    self.velocidade_angular_y_list[i] if i < len(self.velocidade_angular_y_list) else 0,
                    self.velocidade_x_list[i] if i < len(self.velocidade_x_list) else 0,
                    self.velocidade_y_list[i] if i < len(self.velocidade_y_list) else 0,
                    self.posicao_error_x_list[i] if i < len(self.posicao_error_x_list) else 0,
                    self.posicao_error_y_list[i] if i < len(self.posicao_error_y_list) else 0
                ])
    
    def save_phase_data(self):
        """Salva dados separados por fase"""
        phases = [
            ('ROTACIONAR_X', self.timestamps_rot_x, self.x_positions_rot_x, self.y_positions_rot_x,
             self.theta_rads_rot_x, self.theta_degs_rot_x),
            ('MOVER_X', self.timestamps_move_x, self.x_positions_move_x, self.y_positions_move_x,
             self.theta_rads_move_x, self.theta_degs_move_x),
            ('ROTACIONAR_Y', self.timestamps_rot_y, self.x_positions_rot_y, self.y_positions_rot_y,
             self.theta_rads_rot_y, self.theta_degs_rot_y),
            ('MOVER_Y', self.timestamps_move_y, self.x_positions_move_y, self.y_positions_move_y,
             self.theta_rads_move_y, self.theta_degs_move_y)
        ]
        
        for phase_name, time_data, x_data, y_data, theta_rad_data, theta_deg_data in phases:
            if time_data:  # Só salva se houver dados
                filename = os.path.join(self.save_dir, f"{self.experiment_name}_{phase_name}.csv")
                
                with open(filename, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(['time', 'x', 'y', 'theta_rad', 'theta_deg'])
                    
                    for i in range(len(time_data)):
                        writer.writerow([
                            time_data[i],
                            x_data[i],
                            y_data[i],
                            theta_rad_data[i],
                            theta_deg_data[i]
                        ])
    
    def json_serializer(self, obj):
        """Helper para serializar tipos numpy no JSON"""
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, np.float32) or isinstance(obj, np.float64):
            return float(obj)
        if isinstance(obj, np.int32) or isinstance(obj, np.int64):
            return int(obj)
        raise TypeError(f"Type {type(obj)} not serializable")
    
    def save_experiment_summary(self):
        """Salva um resumo de todos os experimentos em um arquivo"""
        summary_file = os.path.join(self.save_dir, "experiments_summary.csv")
        
        # Verificar se o arquivo já existe
        file_exists = os.path.isfile(summary_file)
        
        with open(summary_file, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Escrever cabeçalho se for novo arquivo
            if not file_exists:
                writer.writerow([
                    'experiment_name', 'timestamp', 'target_x', 'target_y',
                    'Kp_angular', 'Kp_linear', 'total_time', 'total_samples',
                    'samples_rot_x', 'samples_move_x', 'samples_rot_y', 'samples_move_y',
                    'success'
                ])
            
            # Escrever dados deste experimento
            writer.writerow([
                self.experiment_name,
                datetime.now().isoformat(),
                self.target_x,
                self.target_y,
                self.Kp_angular,
                self.Kp_linear,
                self.timestamps[-1] if self.timestamps else 0,
                len(self.timestamps),
                len(self.timestamps_rot_x),
                len(self.timestamps_move_x),
                len(self.timestamps_rot_y),
                len(self.timestamps_move_y),
                self.estado == "FINALIZADO"
            ])

###########################################################################################
#ENCERRAMENTO DO NÓ
###########################################################################################

    def iniciar_shutdown_timer(self):
        """Inicia o timer de 2 segundos para encerrar o programa"""
        if not self.program_ending:
            self.program_ending = True
            self.get_logger().info("Destino alcançado! O programa será encerrado em 2 segundos...")
            
            # Criar timer para encerrar após 2 segundos
            self.shutdown_timer = self.create_timer(2.0, self.encerrar_programa)
    
    def encerrar_programa(self):
        """Encerra o programa graciosamente"""
        self.get_logger().info("Encerrando programa...")
        
        # Parar completamente a tartaruga
        vetor_velocidade = Twist()
        vetor_velocidade.linear.x = 0.0
        vetor_velocidade.angular.z = 0.0
        self.velocidade_atuador.publish(vetor_velocidade)

        # Salvar todos os dados
        self.save_all_data()
        
        # Salvar resumo do experimento
        self.save_experiment_summary()
        
        # Exibir estatísticas finais
        self.exibir_estatisticas()

        # Marcar que o programa deve encerrar
        self.should_shutdown = True
        
        # Destruir o timer de shutdown para evitar chamadas repetidas
        if self.shutdown_timer:
            self.shutdown_timer.destroy()
        
        self.get_logger().info("Node pronto para encerramento...")
    
    def exibir_estatisticas(self):
        """Exibe estatísticas do percurso"""
        if self.timestamps:
            tempo_total = self.timestamps[-1]
            self.get_logger().info(f"Tempo total de execução: {tempo_total:.2f} segundos")
            self.get_logger().info(f"Total de amostras coletadas: {len(self.timestamps)}")
            self.get_logger().info(f"Frequência média de amostragem: {len(self.timestamps)/tempo_total:.2f} Hz")

            
            # Mostrar também estatísticas por fase
            if self.x_positions_rot_x:
                self.get_logger().info(f"Amostras ROTACIONAR_X: {len(self.x_positions_rot_x)}")
            if self.x_positions_move_x:
                self.get_logger().info(f"Amostras MOVER_X: {len(self.x_positions_move_x)}")
            if self.x_positions_rot_y:
                self.get_logger().info(f"Amostras ROTACIONAR_Y: {len(self.x_positions_rot_y)}")
            if self.x_positions_move_y:
                self.get_logger().info(f"Amostras MOVER_Y: {len(self.x_positions_move_y)}")

###########################################################################################
#EXECUÇÃO DO CONTROLE
###########################################################################################

    def callback_controle(self, posicao: Pose):

        #Operação com as variáveis de tempo
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        if current_time - self.last_sample_time >= self.sample_interval:

            # Calcular theta em graus
            theta_deg = np.rad2deg(posicao.theta)
            
            # Armazenando os dados GERAIS nos vetores
            self.x_positions.append(posicao.x)
            self.y_positions.append(posicao.y)
            self.theta_rads.append(posicao.theta)
            self.theta_degs.append(theta_deg)
            self.timestamps.append(elapsed_time)
            self.estados.append(self.estado)
            
            # Armazenando dados ESPECÍFICOS por fase
            if self.estado == "ROTACIONAR_X":
                self.x_positions_rot_x.append(posicao.x)
                self.y_positions_rot_x.append(posicao.y)
                self.theta_rads_rot_x.append(posicao.theta)
                self.theta_degs_rot_x.append(theta_deg)
                self.timestamps_rot_x.append(elapsed_time)
            elif self.estado == "MOVER_X":
                self.x_positions_move_x.append(posicao.x)
                self.y_positions_move_x.append(posicao.y)
                self.theta_rads_move_x.append(posicao.theta)
                self.theta_degs_move_x.append(theta_deg)
                self.timestamps_move_x.append(elapsed_time)
            elif self.estado == "ROTACIONAR_Y":
                self.x_positions_rot_y.append(posicao.x)
                self.y_positions_rot_y.append(posicao.y)
                self.theta_rads_rot_y.append(posicao.theta)
                self.theta_degs_rot_y.append(theta_deg)
                self.timestamps_rot_y.append(elapsed_time)
            elif self.estado == "MOVER_Y":
                self.x_positions_move_y.append(posicao.x)
                self.y_positions_move_y.append(posicao.y)
                self.theta_rads_move_y.append(posicao.theta)
                self.theta_degs_move_y.append(theta_deg)
                self.timestamps_move_y.append(elapsed_time)

            # SALVAR DADOS DE CONTROLE
            self.theta_error_x_list.append(self.theta_error_x)
            self.theta_error_y_list.append(self.theta_error_y)
            self.velocidade_angular_x_list.append(self.velocidade_angular_x)
            self.velocidade_angular_y_list.append(self.velocidade_angular_y)
            self.velocidade_x_list.append(self.velocidade_x)
            self.velocidade_y_list.append(self.velocidade_y)
            self.posicao_error_x_list.append(self.posicao_error_x)
            self.posicao_error_y_list.append(self.posicao_error_y)

            # Publicando a posição da tartaruga na tela do computador
            self.get_logger().info(f"x position: {posicao.x:.2f}") 
            self.get_logger().info(f"y position: {posicao.y:.2f}") 
            self.get_logger().info(f"theta [rad]: {posicao.theta:.2f}") 
            self.get_logger().info(f"theta [deg]: {theta_deg:.2f}")
            self.get_logger().info(f"time [s]: {elapsed_time:.2f}")
            self.get_logger().info(f"estado: {self.estado}")
            self.get_logger().info("---")
            
            # Atualiza o tempo da última amostra
            self.last_sample_time = current_time

        #Limite de velocidade
        max_linear_speed = 1.0
        max_angular_speed = 1.0

        # Criando o vetor com os dados de velocidade
        vetor_velocidade = Twist()

        #Orientação da tartaruga para apontar no sentido de x_target
        if posicao.x < self.target_x:
            self.theta_d_rad_x = np.deg2rad(0)
        elif posicao.x > self.target_x:
            self.theta_d_rad_x = np.deg2rad(180)
        else:
            self.theta_d_rad_x = posicao.theta  # Já está na posição x

        #Orientação da tartaruga para apontar no sentido de y_target
        if posicao.y < self.target_y:
            self.theta_d_rad_y = np.deg2rad(90)
        elif posicao.y > self.target_y:
            self.theta_d_rad_y = np.deg2rad(-90)
        else:
            self.theta_d_rad_y = posicao.theta  # Já está na posição y
        
        if self.estado == "ROTACIONAR_X": #Controle de posição angular para orientação a x
            # Verificar se já está na posição X
            if abs(posicao.x - self.target_x) < self.position_tolerance:
                self.estado = "ROTACIONAR_Y"
                self.get_logger().info("Já está na posição X. Indo para rotação Y...")
                return
            
            self.theta_error_x = self.theta_d_rad_x-posicao.theta
            if abs(self.theta_error_x) > self.angle_tolerance:
                self.velocidade_angular_x = self.Kp_angular*self.theta_error_x
                vetor_velocidade.angular.z = self.velocidade_angular_x
                vetor_velocidade.angular.z = np.clip(vetor_velocidade.angular.z, -max_angular_speed, max_angular_speed)
                self.angle_corrected = False                
            else: # Ângulo correto, parar rotação e mudar de estado
                self.velocidade_angular_x = 0.0
                vetor_velocidade.angular.z = self.velocidade_angular_x
                if not self.angle_corrected:
                    self.angle_corrected = True
                    time.sleep(0.5)  # Pequena pausa para estabilização
                    self.estado = "MOVER_X"
                    self.get_logger().info("Ângulo X corrigido. Iniciando movimento em X...")
        
        elif self.estado == "MOVER_X": #Controle de posição linear a x
            self.posicao_error_x = self.target_x - posicao.x
            if abs(self.posicao_error_x) > self.position_tolerance:
                self.velocidade_x = self.Kp_linear*abs(self.posicao_error_x)
                vetor_velocidade.linear.x = self.velocidade_x
                vetor_velocidade.linear.x = np.clip(vetor_velocidade.linear.x , -max_linear_speed, max_linear_speed)
            else: # Chegou na posição X desejada
                self.velocidade_x = 0.0
                vetor_velocidade.linear.x = self.velocidade_x
                self.x_reached = True
                time.sleep(0.5)  # Pequena pausa
                self.estado = "ROTACIONAR_Y"
                self.get_logger().info("Posição X alcançada. Iniciando rotação para Y...")

        elif self.estado == "ROTACIONAR_Y": #Controle de posição angular para orientação a y
            # Verificar se já está na posição Y
            if abs(posicao.y - self.target_y) < self.position_tolerance:
                self.estado = "FINALIZADO"
                self.get_logger().info("Já está na posição Y. Finalizando...")
                return
            
            self.theta_error_y = self.theta_d_rad_y-posicao.theta
            if abs(self.theta_error_y) > self.angle_tolerance:
                self.velocidade_angular_y = self.Kp_angular*self.theta_error_y
                vetor_velocidade.angular.z = self.velocidade_angular_y
                vetor_velocidade.angular.z = np.clip(vetor_velocidade.angular.z, -max_angular_speed, max_angular_speed)
                self.angle_corrected = False                
            else: # Ângulo correto, parar rotação e mudar de estado
                self.velocidade_angular_y = 0.0
                vetor_velocidade.angular.z = self.velocidade_angular_y
                if not self.angle_corrected:
                    self.angle_corrected = True
                    time.sleep(0.5)  # Pequena pausa para estabilização
                    self.estado = "MOVER_Y"
                    self.get_logger().info("Ângulo Y corrigido. Iniciando movimento em Y...")

        elif self.estado == "MOVER_Y": #Controle de posição linear a y
            self.posicao_error_y = self.target_y - posicao.y
            if abs(self.posicao_error_y) > self.position_tolerance:
                self.velocidade_y = self.Kp_linear*abs(self.posicao_error_y)
                vetor_velocidade.linear.x = self.velocidade_y
                vetor_velocidade.linear.x = np.clip(vetor_velocidade.linear.x , -max_linear_speed, max_linear_speed)
            else: # Chegou na posição X desejada
                self.velocidade_y = 0.0
                vetor_velocidade.linear.x = self.velocidade_y 
                self.estado = "FINALIZADO"
                self.get_logger().info("Posição Y alcançada. Destino final alcançado!")
                #Iniciar processo de encerramento
                self.iniciar_shutdown_timer()

        elif self.estado == "FINALIZADO":
            # Parar completamente
            vetor_velocidade.linear.x = 0.0
            vetor_velocidade.angular.z = 0.0
        
        # Publicando a velocidade
        self.velocidade_atuador.publish(vetor_velocidade)

###########################################################################################
# FUNÇÃO PARA OBTER ENTRADAS DO USUÁRIO
###########################################################################################
def get_user_input():
    """Obtém a posição alvo, ganhos e nome do experimento"""
    print("\n" + "="*60)
    print("CONTROLE DA TARTARUGA - ROS2 TURTLESIM")
    print("="*60)
    print("Digite a posição desejada e os ganhos do controlador")
    print("Formato: x y Kp_angular Kp_linear [nome_experimento]")
    print("\nExemplos:")
    print("  5.0 8.0 1.5 1.0                (nome automático)")
    print("  5.0 8.0 1.5 1.0 teste_estavel  (nome personalizado)")
    print("\nPosição: valores entre 0 e 11")
    print("Ganhos: valores positivos (sugestão: 0.5 a 3.0)")
    print("="*60)
    
    while True:
        try:
            user_input = input("\nDigite os valores: ").strip()
            
            if not user_input:
                print("Por favor, digite pelo menos quatro valores.")
                continue
                
            values = user_input.split()
            
            if len(values) < 4:
                print("Erro: Digite pelo menos quatro valores (x y Kp_angular Kp_linear)")
                continue
            
            # Primeiros 4 valores obrigatórios
            x = float(values[0])
            y = float(values[1])
            Kp_angular = float(values[2])
            Kp_linear = float(values[3])
            
            # Nome do experimento (opcional)
            experiment_name = None
            if len(values) >= 5:
                experiment_name = values[4]
            
            # Validar valores
            if x < 0 or x > 11 or y < 0 or y > 11:
                print("Aviso: Valores de posição devem estar entre 0 e 11")
                print("Deseja continuar mesmo assim? (s/n)")
                confirm = input("> ").lower()
                if confirm != 's':
                    continue
            
            if Kp_angular <= 0 or Kp_linear <= 0:
                print("Aviso: Ganhos devem ser valores positivos")
                print("Deseja continuar mesmo assim? (s/n)")
                confirm = input("> ").lower()
                if confirm != 's':
                    continue
            
            print(f"\nConfiguração definida:")
            print(f"  Posição alvo: x={x:.2f}, y={y:.2f}")
            print(f"  Ganho angular (Kp_angular): {Kp_angular:.2f}")
            print(f"  Ganho linear (Kp_linear): {Kp_linear:.2f}")
            if experiment_name:
                print(f"  Nome do experimento: {experiment_name}")
            print("\nIniciando controle...")
            return x, y, Kp_angular, Kp_linear, experiment_name
            
        except ValueError:
            print("Erro: Digite números válidos.")
        except KeyboardInterrupt:
            print("\n\nPrograma cancelado pelo usuário.")
            exit(0)


###########################################################################################
# FUNÇÃO MAIN PARA EXECUÇÃO DO NÓ
###########################################################################################
def main(args=None):
    # Obtendo a posição, ganhos e nome do experimento
    target_x, target_y, Kp_angular, Kp_linear, experiment_name = get_user_input()

    rclpy.init(args=args)
    node = ControlTurtle(target_x, target_y, Kp_angular, Kp_linear, experiment_name)
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    print("\nControlador iniciado. Aguardando conclusão...")
    print(f"Dados serão salvos em: turtle_experiments/{experiment_name or node.experiment_name}*")
    
    try:
        while rclpy.ok() and not node.should_shutdown:
            executor.spin_once(timeout_sec=0.1)
            
        if node.should_shutdown:
            print("\n" + "="*60)
            print("PROGRAMA CONCLUÍDO COM SUCESSO!")
            print("="*60)
            print(f"Dados salvos em: turtle_experiments/")
            print(f"Arquivos gerados:")
            print(f"  • {node.experiment_name}.json (dados completos)")
            print(f"  • {node.experiment_name}_main.csv (tabela principal)")
            print(f"  • {node.experiment_name}_*.csv (dados por fase)")
            print(f"  • experiments_summary.csv (resumo de todos experimentos)")
            print("="*60)
            
    except KeyboardInterrupt:
        print("\nPrograma interrompido pelo usuário.")
        # Salvar dados mesmo se interrompido
        node.save_all_data()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
        print("Node encerrado com sucesso!")

if __name__ == '__main__':
    main()
