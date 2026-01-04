#!/usr/bin/env python3
import json
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams

# Configurações de plotagem para melhor visualização
rcParams.update({
    'figure.autolayout': True,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'axes.labelsize': 12,
    'axes.titlesize': 14,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 10,
    'font.size': 11,
    'lines.linewidth': 3.5,
    'lines.markersize': 6,
})

def load_experiment_data(experiment_prefix):
    """
    Carrega dados de um experimento baseado no prefixo (kp1, kp2, etc.)
    Retorna um dicionário com os dados processados
    """
    # Encontrar arquivos com o prefixo
    json_files = glob.glob(f"turtle_experiments/{experiment_prefix}*.json")
    
    if not json_files:
        # Tentar encontrar sem o diretório
        json_files = glob.glob(f"{experiment_prefix}*.json")
    
    if not json_files:
        raise FileNotFoundError(f"Nenhum arquivo encontrado para o prefixo {experiment_prefix}")
    
    # Usar o primeiro arquivo encontrado
    json_file = json_files[0]
    
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    # Extrair dados relevantes
    metadata = data['metadata']
    experiment_data = data['data']
    
    # Converter listas para numpy arrays
    time_data = np.array(experiment_data['time'])
    x_data = np.array(experiment_data['x'])
    y_data = np.array(experiment_data['y'])
    theta_rad_data = np.array(experiment_data['theta_rad'])
    theta_deg_data = np.array(experiment_data['theta_deg'])
    
    # Extrair velocidades (pode não existir em todos os arquivos)
    linear_vel_x = np.array(experiment_data.get('linear_vel_x', [0]*len(time_data)))
    linear_vel_y = np.array(experiment_data.get('linear_vel_y', [0]*len(time_data)))
    angular_vel_x = np.array(experiment_data.get('angular_vel_x', [0]*len(time_data)))
    angular_vel_y = np.array(experiment_data.get('angular_vel_y', [0]*len(time_data)))
    
    # Velocidade angular total (z)
    angular_vel = angular_vel_x + angular_vel_y  # Ou pegar apenas uma componente
    
    # Valores alvo
    target_x = metadata['target_position']['x']
    target_y = metadata['target_position']['y']
    
    # Ganho Kp
    Kp_angular = metadata['controller_gains']['Kp_angular']
    Kp_linear = metadata['controller_gains']['Kp_linear']
    
    return {
        'experiment_name': metadata['experiment_name'],
        'prefix': experiment_prefix,
        'time': time_data,
        'x': x_data,
        'y': y_data,
        'theta_rad': theta_rad_data,
        'theta_deg': theta_deg_data,
        'linear_vel_x': linear_vel_x,
        'linear_vel_y': linear_vel_y,
        'angular_vel': angular_vel,
        'target_x': target_x,
        'target_y': target_y,
        'Kp_angular': Kp_angular,
        'Kp_linear': Kp_linear,
        'total_samples': len(time_data),
        'total_time': time_data[-1] if len(time_data) > 0 else 0
    }

def extend_data_to_max_time(data, max_time):
    """
    Estende os dados de um experimento para o tempo máximo,
    mantendo o último valor constante
    """
    original_time = data['time']
    original_x = data['x']
    original_y = data['y']
    original_theta_rad = data['theta_rad']
    original_theta_deg = data['theta_deg']
    original_linear_vel_x = data['linear_vel_x']
    original_linear_vel_y = data['linear_vel_y']
    original_angular_vel = data['angular_vel']
    
    # Se já tem o tempo máximo, retorna os dados originais
    if original_time[-1] >= max_time:
        return data
    
    # Criar novo vetor de tempo até max_time
    # Manter a mesma frequência de amostragem estimada
    if len(original_time) > 1:
        avg_sample_interval = original_time[-1] / len(original_time)
        num_additional_samples = int((max_time - original_time[-1]) / avg_sample_interval)
        
        if num_additional_samples > 0:
            # Criar tempo estendido
            extended_time = np.concatenate([
                original_time,
                np.linspace(original_time[-1] + avg_sample_interval, max_time, num_additional_samples)
            ])
            
            # Estender dados mantendo último valor constante
            extended_x = np.concatenate([original_x, np.full(num_additional_samples, original_x[-1])])
            extended_y = np.concatenate([original_y, np.full(num_additional_samples, original_y[-1])])
            extended_theta_rad = np.concatenate([original_theta_rad, np.full(num_additional_samples, original_theta_rad[-1])])
            extended_theta_deg = np.concatenate([original_theta_deg, np.full(num_additional_samples, original_theta_deg[-1])])
            extended_linear_vel_x = np.concatenate([original_linear_vel_x, np.zeros(num_additional_samples)])
            extended_linear_vel_y = np.concatenate([original_linear_vel_y, np.zeros(num_additional_samples)])
            extended_angular_vel = np.concatenate([original_angular_vel, np.zeros(num_additional_samples)])
            
            return {
                **data,
                'time': extended_time,
                'x': extended_x,
                'y': extended_y,
                'theta_rad': extended_theta_rad,
                'theta_deg': extended_theta_deg,
                'linear_vel_x': extended_linear_vel_x,
                'linear_vel_y': extended_linear_vel_y,
                'angular_vel': extended_angular_vel
            }
    
    return data

def plot_comparison_graphs(experiments_data, max_time):
    """
    Cria os 6 gráficos de comparação entre experimentos
    """
    # Cores para diferentes experimentos
    colors = ['blue', 'red', 'green', 'orange', 'purple', 'brown']
    markers = ['o', 's', '^', 'D', 'v', '<']
    
    # Verificar se temos valores alvo (devem ser os mesmos para todos os experimentos)
    target_x = experiments_data[0]['target_x']
    target_y = experiments_data[0]['target_y']
    
    # Criar figura com 6 subplots
    fig, axes = plt.subplots(3, 2, figsize=(16, 12))
    fig.suptitle('Comparação entre Experimentos com Diferentes Ganhos Kp', fontsize=16, fontweight='bold')
    
    # GRÁFICO 1: Posição X
    ax1 = axes[0, 0]
    for i, exp in enumerate(experiments_data):
        label = f"{exp['prefix']} (Kp={exp['Kp_linear']:.2f})"
        ax1.plot(exp['time'], exp['x'], color=colors[i], marker=markers[i], markevery=0.1, label=label, alpha=0.8)
    
    # Linha de referência (target)
    ax1.axhline(y=target_x, color='black', linestyle='--', linewidth=3.5, label=f'Target X = {target_x:.2f}', alpha=0.7)
    
    ax1.set_xlabel('Tempo (s)')
    ax1.set_ylabel('Posição X (m)')
    ax1.set_title('Posição X vs Tempo')
    ax1.legend(loc='best')
    ax1.set_xlim([0, max_time])
    ax1.grid(True, alpha=0.3)
    
    # GRÁFICO 2: Posição Y
    ax2 = axes[0, 1]
    for i, exp in enumerate(experiments_data):
        label = f"{exp['prefix']} (Kp={exp['Kp_linear']:.2f})"
        ax2.plot(exp['time'], exp['y'], color=colors[i], marker=markers[i], markevery=0.1, label=label, alpha=0.8)
    
    # Linha de referência (target)
    ax2.axhline(y=target_y, color='black', linestyle='--', linewidth=3.5, label=f'Target Y = {target_y:.2f}', alpha=0.7)
    
    ax2.set_xlabel('Tempo (s)')
    ax2.set_ylabel('Posição Y (m)')
    ax2.set_title('Posição Y vs Tempo')
    ax2.legend(loc='best')
    ax2.set_xlim([0, max_time])
    ax2.grid(True, alpha=0.3)
    
    # GRÁFICO 3: Velocidade Linear X
    ax3 = axes[1, 0]
    for i, exp in enumerate(experiments_data):
        label = f"{exp['prefix']} (Kp={exp['Kp_linear']:.2f})"
        ax3.plot(exp['time'], exp['linear_vel_x'], color=colors[i], marker=markers[i], markevery=0.1, label=label, alpha=0.8)
    
    ax3.set_xlabel('Tempo (s)')
    ax3.set_ylabel('Velocidade Linear X (m/s)')
    ax3.set_title('Velocidade Linear X vs Tempo')
    ax3.legend(loc='best')
    ax3.set_xlim([0, max_time])
    ax3.grid(True, alpha=0.3)
    
    # GRÁFICO 4: Velocidade Linear Y
    ax4 = axes[1, 1]
    for i, exp in enumerate(experiments_data):
        label = f"{exp['prefix']} (Kp={exp['Kp_linear']:.2f})"
        ax4.plot(exp['time'], exp['linear_vel_y'], color=colors[i], marker=markers[i], markevery=0.1, label=label, alpha=0.8)
    
    ax4.set_xlabel('Tempo (s)')
    ax4.set_ylabel('Velocidade Linear Y (m/s)')
    ax4.set_title('Velocidade Linear Y vs Tempo')
    ax4.legend(loc='best')
    ax4.set_xlim([0, max_time])
    ax4.grid(True, alpha=0.3)
    
    # GRÁFICO 5: Posição Angular (em graus)
    ax5 = axes[2, 0]
    for i, exp in enumerate(experiments_data):
        label = f"{exp['prefix']} (Kp_angular={exp['Kp_angular']:.2f})"
        ax5.plot(exp['time'], exp['theta_deg'], color=colors[i], marker=markers[i], markevery=0.1, label=label, alpha=0.8)
    
    ax5.set_xlabel('Tempo (s)')
    ax5.set_ylabel('Ângulo (graus)')
    ax5.set_title('Posição Angular vs Tempo')
    ax5.legend(loc='best')
    ax5.set_xlim([0, max_time])
    ax5.grid(True, alpha=0.3)
    
    # GRÁFICO 6: Velocidade Angular
    ax6 = axes[2, 1]
    for i, exp in enumerate(experiments_data):
        label = f"{exp['prefix']} (Kp_angular={exp['Kp_angular']:.2f})"
        ax6.plot(exp['time'], exp['angular_vel'], color=colors[i], marker=markers[i], markevery=0.1, label=label, alpha=0.8)
    
    ax6.set_xlabel('Tempo (s)')
    ax6.set_ylabel('Velocidade Angular (rad/s)')
    ax6.set_title('Velocidade Angular vs Tempo')
    ax6.legend(loc='best')
    ax6.set_xlim([0, max_time])
    ax6.grid(True, alpha=0.3)
    
    # Ajustar layout
    plt.tight_layout()
    plt.subplots_adjust(top=0.93)
    
    return fig, colors

def create_summary_table(experiments_data):
    """
    Cria uma tabela resumo dos experimentos
    """
    print("\n" + "="*80)
    print("RESUMO DOS EXPERIMENTOS")
    print("="*80)
    print(f"{'Experimento':<15} {'Kp_linear':<10} {'Kp_angular':<12} {'Tempo Final (s)':<15} {'Amostras':<10}")
    print("-"*80)
    
    for exp in experiments_data:
        print(f"{exp['prefix']:<15} {exp['Kp_linear']:<10.2f} {exp['Kp_angular']:<12.2f} {exp['total_time']:<15.2f} {exp['total_samples']:<10}")
    
    print("="*80)

def main():
    # Lista de prefixos dos experimentos
    experiment_prefixes = ['kp1', 'kp2', 'kp3', 'kp4']
    
    # Carregar dados de todos os experimentos
    experiments_data = []
    max_time = 0
    
    print("Carregando dados dos experimentos...")
    for prefix in experiment_prefixes:
        try:
            data = load_experiment_data(prefix)
            experiments_data.append(data)
            print(f"  ✓ {prefix}: {data['experiment_name']} - {data['total_time']:.2f}s, {data['total_samples']} amostras")
            
            # Atualizar tempo máximo
            if data['total_time'] > max_time:
                max_time = data['total_time']
                
        except FileNotFoundError as e:
            print(f"  ✗ {prefix}: {e}")
        except Exception as e:
            print(f"  ✗ {prefix}: Erro ao carregar - {e}")
    
    if not experiments_data:
        print("Nenhum dado de experimento carregado. Verifique os arquivos.")
        return
    
    print(f"\nTempo máximo entre experimentos: {max_time:.2f} segundos")
    
    # Estender dados de experimentos mais curtos para o tempo máximo
    print("\nProcessando dados para tempo comum...")
    for i in range(len(experiments_data)):
        if experiments_data[i]['total_time'] < max_time:
            experiments_data[i] = extend_data_to_max_time(experiments_data[i], max_time)
            print(f"  ✓ {experiments_data[i]['prefix']}: Estendido para {max_time:.2f}s")
    
    # Criar tabela resumo
    create_summary_table(experiments_data)
    
    # Plotar gráficos de comparação
    print("\nGerando gráficos de comparação...")
    fig, colors = plot_comparison_graphs(experiments_data, max_time)
    
    # Salvar figura
    output_filename = "comparacao_experimentos.png"
    fig.savefig(output_filename, dpi=300, bbox_inches='tight')
    print(f"\n✓ Gráficos salvos como '{output_filename}'")
    
    # Mostrar gráficos
    plt.show()
    
    # Plotar também gráficos individuais maiores
    print("\nGerando gráficos individuais em alta resolução...")
    
    # Lista de gráficos individuais a criar
    plot_configs = [
        ('posicao_x.png', 'Posição X vs Tempo', 'Tempo (s)', 'Posição X (m)', 
         lambda exp: (exp['time'], exp['x']), experiments_data[0]['target_x']),
        ('posicao_y.png', 'Posição Y vs Tempo', 'Tempo (s)', 'Posição Y (m)', 
         lambda exp: (exp['time'], exp['y']), experiments_data[0]['target_y']),
        ('velocidade_x.png', 'Velocidade Linear X vs Tempo', 'Tempo (s)', 'Velocidade X (m/s)', 
         lambda exp: (exp['time'], exp['linear_vel_x']), None),
        ('velocidade_y.png', 'Velocidade Linear Y vs Tempo', 'Tempo (s)', 'Velocidade Y (m/s)', 
         lambda exp: (exp['time'], exp['linear_vel_y']), None),
        ('angulo.png', 'Ângulo vs Tempo', 'Tempo (s)', 'Ângulo (graus)', 
         lambda exp: (exp['time'], exp['theta_deg']), None),
        ('velocidade_angular.png', 'Velocidade Angular vs Tempo', 'Tempo (s)', 'Velocidade Angular (rad/s)', 
         lambda exp: (exp['time'], exp['angular_vel']), None),
    ]
    
    for filename, title, xlabel, ylabel, data_func, target_line in plot_configs:
        fig_ind, ax_ind = plt.subplots(figsize=(12, 8))
        
        for i, exp in enumerate(experiments_data):
            x_data, y_data = data_func(exp)
            label = f"{exp['prefix']} (Kp_linear={exp['Kp_linear']:.2f}, Kp_angular={exp['Kp_angular']:.2f})"
            ax_ind.plot(x_data, y_data, color=colors[i], linewidth=3.5, label=label, alpha=0.8)
        
        if target_line is not None:
            param_name = 'X' if 'posicao_x' in filename else 'Y'
            ax_ind.axhline(y=target_line, color='black', linestyle='--', linewidth=3.5, 
                          label=f'Target {param_name} = {target_line:.2f}', alpha=0.7)
        
        ax_ind.set_xlabel(xlabel, fontsize=14)
        ax_ind.set_ylabel(ylabel, fontsize=14)
        ax_ind.set_title(title, fontsize=16, fontweight='bold')
        ax_ind.legend(loc='best', fontsize=11)
        ax_ind.grid(True, alpha=0.3)
        ax_ind.set_xlim([0, max_time])
        
        plt.tight_layout()
        fig_ind.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close(fig_ind)
        print(f"  ✓ {filename}")
    
    print("\n✓ Todos os gráficos foram gerados com sucesso!")

if __name__ == '__main__':
    main()