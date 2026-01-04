#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np
import time
import math

###########################################################################################
# CRIANDO O NÓ DE CONTROLE
###########################################################################################
class ControlTurtle(Node):
    def __init__(self, target_x, target_y):
        super().__init__('turtle_control')
        self.posicao_sensor = self.create_subscription(Pose, "turtle1/pose", self.callback_controle, 10)
        self.velocidade_atuador = self.create_publisher(Twist, "turtle1/cmd_vel", 10)

        # Variáveis para armazenamento dos dados
        self.x_positions = []
        self.y_positions = []
        self.theta_rads = []
        self.theta_degs = []
        self.timestamps = []

        # Variáveis de tempo
        self.start_time = time.time()
        self.last_sample_time = self.start_time
        self.sample_interval = 0.001  # tempo para amostragem

        # Variáveis para controle de posição
        self.target_x = target_x
        self.target_y = target_y
        
        # Estados do controlador (abordagem didática)
        self.estado = "ROTACIONAR_X"  # Estados: ROTACIONAR_X, MOVER_X, ROTACIONAR_Y, MOVER_Y, FINALIZADO
        self.angle_tolerance = 0.05  # rad (aproximadamente 2.9 graus)
        self.position_tolerance = 0.05  # tolerância para posição
        
        # Controle de memória para evitar oscilações
        self.angle_corrected = False
        self.x_reached = False

    def callback_controle(self, posicao: Pose):
        # Operação com as variáveis de tempo
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        ###########################################################################################
        # LEITURA DO SENSOR DE POSIÇÃO
        ###########################################################################################
        if current_time - self.last_sample_time >= self.sample_interval:
            # Armazenando os dados nos vetores
            theta_deg = np.rad2deg(posicao.theta)
            self.x_positions.append(posicao.x)
            self.y_positions.append(posicao.y)
            self.theta_rads.append(posicao.theta)
            self.theta_degs.append(theta_deg)
            self.timestamps.append(elapsed_time)

            # Publicando a posição da tartaruga na tela do computador
            self.get_logger().info(f"Estado: {self.estado}")
            self.get_logger().info(f"x position: {posicao.x:.2f}") 
            self.get_logger().info(f"y position: {posicao.y:.2f}") 
            self.get_logger().info(f"theta [rad]: {posicao.theta:.2f}") 
            self.get_logger().info(f"theta [deg]: {theta_deg:.2f}")
            self.get_logger().info(f"time [s]: {elapsed_time:.2f}")
            self.get_logger().info("---")
            
            # Atualiza o tempo da última amostra
            self.last_sample_time = current_time

        ###########################################################################################
        # MÁQUINA DE ESTADOS DO CONTROLADOR (ABORDAGEM DIDÁTICA)
        ###########################################################################################
        # Ganhos do controlador
        Kp_angular = 2.0
        Kp_linear = 1.5
        
        # Limites de velocidade
        max_linear_speed = 2.0
        max_angular_speed = 2.0
        
        # Criando o vetor com os dados de velocidade
        vetor_velocidade = Twist()
        
        # Normalizar o ângulo da tartaruga para o intervalo [-pi, pi]
        current_theta = posicao.theta
        while current_theta > math.pi:
            current_theta -= 2 * math.pi
        while current_theta < -math.pi:
            current_theta += 2 * math.pi
            
        if self.estado == "ROTACIONAR_X":
            # Calcular ângulo desejado para movimento em X
            if posicao.x < self.target_x:
                theta_desejado_rad = 0.0  # Apontar para direita
            else:
                theta_desejado_rad = math.pi  # Apontar para esquerda
            
            # Normalizar ângulo desejado
            while theta_desejado_rad > math.pi:
                theta_desejado_rad -= 2 * math.pi
            while theta_desejado_rad < -math.pi:
                theta_desejado_rad += 2 * math.pi
            
            # Calcular erro angular (menor diferença)
            theta_error_x = theta_desejado_rad - current_theta
            while theta_error_x > math.pi:
                theta_error_x -= 2 * math.pi
            while theta_error_x < -math.pi:
                theta_error_x += 2 * math.pi
            
            self.get_logger().info(f"ROTACIONAR_X - Erro angular: {theta_error_x:.3f} rad")
            
            # Se o erro for maior que a tolerância, rotacionar
            if abs(theta_error_x) > self.angle_tolerance:
                vetor_velocidade.angular.z = Kp_angular * theta_error_x
                vetor_velocidade.angular.z = np.clip(vetor_velocidade.angular.z, -max_angular_speed, max_angular_speed)
                self.angle_corrected = False
            else:
                # Ângulo correto, parar rotação e mudar de estado
                vetor_velocidade.angular.z = 0.0
                if not self.angle_corrected:
                    self.angle_corrected = True
                    time.sleep(0.5)  # Pequena pausa para estabilização
                    self.estado = "MOVER_X"
                    self.get_logger().info("Ângulo X corrigido. Iniciando movimento em X...")
        
        elif self.estado == "MOVER_X":
            # Verificar se ainda precisa corrigir o ângulo
            if posicao.x < self.target_x:
                theta_desejado_rad = 0.0
            else:
                theta_desejado_rad = math.pi
                
            theta_error_x = theta_desejado_rad - current_theta
            while theta_error_x > math.pi:
                theta_error_x -= 2 * math.pi
            while theta_error_x < -math.pi:
                theta_error_x += 2 * math.pi
            
            # Calcular erro de posição em X
            posicao_error_x = self.target_x - posicao.x
            
            self.get_logger().info(f"MOVER_X - Erro posição X: {posicao_error_x:.3f}, Erro angular: {theta_error_x:.3f}")
            
            # Se ainda longe do alvo em X
            if abs(posicao_error_x) > self.position_tolerance:
                # Correção angular durante o movimento
                if abs(theta_error_x) > self.angle_tolerance:
                    vetor_velocidade.angular.z = Kp_angular * theta_error_x * 0.5  # Ganho menor durante movimento
                else:
                    vetor_velocidade.angular.z = 0.0
                
                # Velocidade linear (com direção correta)
                linear_speed = Kp_linear * abs(posicao_error_x)
                linear_speed = min(linear_speed, max_linear_speed)
                
                # Determinar direção do movimento
                if posicao_error_x > 0:
                    vetor_velocidade.linear.x = linear_speed  # Movimento para frente
                else:
                    vetor_velocidade.linear.x = -linear_speed  # Movimento para trás
                    
                vetor_velocidade.linear.x = np.clip(vetor_velocidade.linear.x, -max_linear_speed, max_linear_speed)
            else:
                # Chegou na posição X desejada
                vetor_velocidade.linear.x = 0.0
                vetor_velocidade.angular.z = 0.0
                self.x_reached = True
                time.sleep(0.5)  # Pequena pausa
                self.estado = "ROTACIONAR_Y"
                self.get_logger().info("Posição X alcançada. Iniciando rotação para Y...")
        
        elif self.estado == "ROTACIONAR_Y":
            # Calcular ângulo desejado para movimento em Y
            if posicao.y < self.target_y:
                theta_desejado_rad = math.pi/2  # Apontar para cima
            else:
                theta_desejado_rad = -math.pi/2  # Apontar para baixo
            
            # Normalizar ângulo desejado
            while theta_desejado_rad > math.pi:
                theta_desejado_rad -= 2 * math.pi
            while theta_desejado_rad < -math.pi:
                theta_desejado_rad += 2 * math.pi
            
            # Calcular erro angular
            theta_error_y = theta_desejado_rad - current_theta
            while theta_error_y > math.pi:
                theta_error_y -= 2 * math.pi
            while theta_error_y < -math.pi:
                theta_error_y += 2 * math.pi
            
            self.get_logger().info(f"ROTACIONAR_Y - Erro angular: {theta_error_y:.3f} rad")
            
            # Se o erro for maior que a tolerância, rotacionar
            if abs(theta_error_y) > self.angle_tolerance:
                vetor_velocidade.angular.z = Kp_angular * theta_error_y
                vetor_velocidade.angular.z = np.clip(vetor_velocidade.angular.z, -max_angular_speed, max_angular_speed)
                self.angle_corrected = False
            else:
                # Ângulo correto, parar rotação e mudar de estado
                vetor_velocidade.angular.z = 0.0
                if not self.angle_corrected:
                    self.angle_corrected = True
                    time.sleep(0.5)  # Pequena pausa para estabilização
                    self.estado = "MOVER_Y"
                    self.get_logger().info("Ângulo Y corrigido. Iniciando movimento em Y...")
        
        elif self.estado == "MOVER_Y":
            # Verificar se ainda precisa corrigir o ângulo
            if posicao.y < self.target_y:
                theta_desejado_rad = math.pi/2
            else:
                theta_desejado_rad = -math.pi/2
                
            theta_error_y = theta_desejado_rad - current_theta
            while theta_error_y > math.pi:
                theta_error_y -= 2 * math.pi
            while theta_error_y < -math.pi:
                theta_error_y += 2 * math.pi
            
            # Calcular erro de posição em Y
            posicao_error_y = self.target_y - posicao.y
            
            self.get_logger().info(f"MOVER_Y - Erro posição Y: {posicao_error_y:.3f}, Erro angular: {theta_error_y:.3f}")
            
            # Se ainda longe do alvo em Y
            if abs(posicao_error_y) > self.position_tolerance:
                # Correção angular durante o movimento
                if abs(theta_error_y) > self.angle_tolerance:
                    vetor_velocidade.angular.z = Kp_angular * theta_error_y * 0.5  # Ganho menor durante movimento
                else:
                    vetor_velocidade.angular.z = 0.0
                
                # Velocidade linear (com direção correta)
                linear_speed = Kp_linear * abs(posicao_error_y)
                linear_speed = min(linear_speed, max_linear_speed)
                
                # Determinar direção do movimento
                if posicao_error_y > 0:
                    vetor_velocidade.linear.x = linear_speed  # Movimento para frente
                else:
                    vetor_velocidade.linear.x = -linear_speed  # Movimento para trás
                    
                vetor_velocidade.linear.x = np.clip(vetor_velocidade.linear.x, -max_linear_speed, max_linear_speed)
            else:
                # Chegou na posição Y desejada
                vetor_velocidade.linear.x = 0.0
                vetor_velocidade.angular.z = 0.0
                self.estado = "FINALIZADO"
                self.get_logger().info("Posição Y alcançada. Destino final alcançado!")
        
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
    """Obtém a posição alvo do usuário antes de iniciar o nó"""
    print("\n" + "="*50)
    print("CONTROLE DA TARTARUGA - ROS2 TURTLESIM")
    print("="*50)
    print("CONTROLE DIDÁTICO: Primeiro em X, depois em Y")
    print("Digite a posição desejada para a tartaruga")
    print("Valores devem estar entre 0 e 11")
    print("="*50)
    
    while True:
        try:
            user_input = input("\nDigite a posição (formato: x y): ").strip()
            
            if not user_input:
                print("Por favor, digite dois valores.")
                continue
                
            values = user_input.split()
            
            if len(values) != 2:
                print("Erro: Digite exatamente dois valores (x y)")
                print("Exemplo: 3.5 8.0")
                continue
                
            x = float(values[0])
            y = float(values[1])
            
            # Validar valores (turtlesim vai de 0 a 11)
            if x < 0 or x > 11 or y < 0 or y > 11:
                print("Aviso: Valores devem estar entre 0 e 11")
                print("Deseja continuar mesmo assim? (s/n)")
                confirm = input("> ").lower()
                if confirm != 's':
                    continue
            
            print(f"\nPosição alvo definida: x={x:.2f}, y={y:.2f}")
            print("ESTRATÉGIA DIDÁTICA:")
            print("1. Rotacionar para apontar no eixo X")
            print("2. Mover até a posição X desejada")
            print("3. Rotacionar para apontar no eixo Y")
            print("4. Mover até a posição Y desejada")
            print("Iniciando controle...")
            return x, y
            
        except ValueError:
            print("Erro: Digite números válidos. Exemplo: '5.5 2.0'")
        except KeyboardInterrupt:
            print("\n\nPrograma cancelado pelo usuário.")
            exit(0)
        except Exception as e:
            print(f"Erro: {e}")


###########################################################################################
# FUNÇÃO MAIN PARA EXECUÇÃO DO NÓ
###########################################################################################
def main(args=None):
    # Obtendo a nova posição da tartaruga
    target_x, target_y = get_user_input()

    rclpy.init(args=args)
    node = ControlTurtle(target_x, target_y)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()