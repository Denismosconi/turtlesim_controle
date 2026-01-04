#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np
import time
import math

###########################################################################################
#CRIANDO O NÓ DE CONTROLE
###########################################################################################
class ControlTurtle(Node):
    def __init__(self, target_x, target_y, Kp_angular, Kp_linear):
        super().__init__('turtle_control')
        self.posicao_sensor = self.create_subscription(Pose, "turtle1/pose", self.callback_controle, 10)
        self.velocidade_atuador = self.create_publisher(Twist, "turtle1/cmd_vel", 10)

        # Variáveis para armazenamento dos dados
        self.x_positions = []
        self.y_positions = []
        self.theta_rads = []
        self.theta_degs = []
        self.timestamps = []
        self.estados = []  # Para registrar o estado em cada amostra
        
        # NOVOS VETORES PARA DADOS DE CONTROLE
        self.theta_error_x_list = []
        self.theta_error_y_list = []
        self.velocidade_angular_x_list = []
        self.velocidade_angular_y_list = []
        self.velocidade_x_list = []
        self.velocidade_y_list = []
        self.posicao_error_x_list = []
        self.posicao_error_y_list = []

        # NOVOS VETORES SEPARADOS POR FASE
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
        self.sample_interval = 0.001  # tempo para amostragem

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

    def callback_controle(self, posicao: Pose):

        #Operação com as variáveis de tempo
        current_time = time.time()
        elapsed_time = current_time - self.start_time

###########################################################################################
# LEITURA DO SENSOR DE POSIÇÃO
###########################################################################################
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

###########################################################################################
# APLICANDO O SINAL DE CONTROLE
###########################################################################################
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
# FUNÇÃO PARA OBTER ENTRADA DO USUÁRIO
###########################################################################################
def get_user_input():
    """Obtém a posição alvo e ganhos do controlador do usuário"""
    print("\n" + "="*60)
    print("CONTROLE DA TARTARUGA - ROS2 TURTLESIM")
    print("="*60)
    print("Digite a posição desejada e os ganhos do controlador")
    print("Formato: x y Kp_angular Kp_linear")
    print("\nExemplo: 5.0 8.0 1.5 1.0")
    print("\nPosição: valores entre 0 e 11")
    print("Ganhos: valores positivos (sugestão: 0.5 a 3.0)")
    print("="*60)
    
    while True:
        try:
            user_input = input("\nDigite os valores (formato: x y Kp_angular Kp_linear): ").strip()
            
            if not user_input:
                print("Por favor, digite quatro valores.")
                continue
                
            values = user_input.split()
            
            if len(values) != 4:
                print("Erro: Digite exatamente quatro valores (x y Kp_angular Kp_linear)")
                print("Exemplo: 3.5 8.0 1.5 1.0")
                continue
                
            x = float(values[0])
            y = float(values[1])
            Kp_angular = float(values[2])
            Kp_linear = float(values[3])
            
            # Validar valores de posição
            if x < 0 or x > 11 or y < 0 or y > 11:
                print("Aviso: Valores de posição devem estar entre 0 e 11")
                print("Deseja continuar mesmo assim? (s/n)")
                confirm = input("> ").lower()
                if confirm != 's':
                    continue
            
            # Validar ganhos
            if Kp_angular <= 0 or Kp_linear <= 0:
                print("Aviso: Ganhos devem ser valores positivos")
                print("Deseja continuar mesmo assim? (s/n)")
                confirm = input("> ").lower()
                if confirm != 's':
                    continue
            
            # Sugestão de valores razoáveis
            if Kp_angular > 5.0 or Kp_linear > 5.0:
                print("Aviso: Ganhos muito altos podem causar instabilidade")
                print("Deseja continuar mesmo assim? (s/n)")
                confirm = input("> ").lower()
                if confirm != 's':
                    continue
            
            print(f"\nConfiguração definida:")
            print(f"  Posição alvo: x={x:.2f}, y={y:.2f}")
            print(f"  Ganho angular (Kp_angular): {Kp_angular:.2f}")
            print(f"  Ganho linear (Kp_linear): {Kp_linear:.2f}")
            print("\nIniciando controle...")
            return x, y, Kp_angular, Kp_linear
            
        except ValueError:
            print("Erro: Digite números válidos. Exemplo: '5.0 8.0 1.5 1.0'")
        except KeyboardInterrupt:
            print("\n\nPrograma cancelado pelo usuário.")
            exit(0)
        except Exception as e:
            print(f"Erro: {e}")


###########################################################################################
# FUNÇÃO MAIN PARA EXECUÇÃO DO NÓ
###########################################################################################
def main(args=None):
    # Obtendo a posição e ganhos da tartaruga
    target_x, target_y, Kp_angular, Kp_linear = get_user_input()

    rclpy.init(args=args)
    node = ControlTurtle(target_x, target_y, Kp_angular, Kp_linear)
    
    # Criar um executor separado para melhor controle
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    print("\nControlador iniciado. Aguardando conclusão...")
    
    try:
        # Loop principal que verifica se o programa deve continuar
        while rclpy.ok() and not node.should_shutdown:
            executor.spin_once(timeout_sec=0.1)
            
        if node.should_shutdown:
            print("\n" + "="*60)
            print("PROGRAMA CONCLUÍDO COM SUCESSO!")
            print("="*60)
            print("O controlador foi encerrado automaticamente.")
            print(f"Posição alvo alcançada: ({target_x}, {target_y})")
            print(f"Ganhos utilizados: Kp_angular={Kp_angular}, Kp_linear={Kp_linear}")
            print("="*60)
            
    except KeyboardInterrupt:
        print("\nPrograma interrompido pelo usuário.")
    finally:
        # Limpeza final
        print("Realizando limpeza final...")
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
        print("Node encerrado com sucesso!")

if __name__ == '__main__':
    main()
