%Este programa plota a resposta da planta (tartaruga) para algumas entradas
%padrão, utiliadas em projetos de sistemas de controle.
%Função de transferência da planta: 1/s
%Entradas analisadas: impulso, degrau unitário e rampa.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%INICIALIZAÇÃO
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Limpando as variáveis e a tela
clear;
close all;
clc;

%Definindo as cores para plotagem
cor1 = [43, 47, 138]*1/255;
cor2 = [178, 89, 94]*1/255;
cor3 = [68, 178, 134]*1/255;
cor4 = [79, 13, 57]*1/255;
cor5 = [16, 202, 115]*1/255;
cor6 = [182, 0, 29]*1/255;
cor7 = [49, 217, 76]*1/255;
cor8 = [0, 127, 195]*1/255;
cor9 = [0, 0, 0]*1/255;
cor10 = [0, 138, 0]*1/255;
cor11 = [179, 57, 81]*1/255;

%Tempo de simulação
t = 0:0.01:3; 

%Criando a função de transferência da planta
num = 1;
den = [1 0];
G = tf(num,den);

fprintf('Função de Transferência:\n');
display(G);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%RESPOSTA AO IMPULSO
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[y,tOut] = impulse(G, t);
figure(1)
plot(tOut, y, 'LineWidth', 3.5, 'Color', cor1);
title('Resposta ao Impulso', 'FontSize', 16);
xlabel('Tempo [s]', 'FontSize', 16);
ylabel('Posição [m]', 'FontSize', 16);
set(gca, 'FontSize', 16);
ylim([0 5]);
xlim([0 t(end)]);
grid on;
print('Figure_1_impulse', '-dpng', '-r300');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%RESPOSTA AO DEGRAU
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[y,tOut] = step(G, t);
figure(2)
plot(tOut, y, 'LineWidth', 3.5, 'Color', cor1);
title('Resposta ao Degrau','FontSize', 16);
xlabel('Tempo [s]', 'FontSize', 16);
ylabel('Posição [m]', 'FontSize', 16);
set(gca, 'FontSize', 16);
ylim([0 5]);
xlim([0 t(end)]);
grid on;
print('Figure_2_step', '-dpng', '-r300');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%RESPOSTA À RAMPA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
s = tf('s');
ramp_response = step(G/s, t);
figure(3)
plot(t, ramp_response, 'LineWidth', 3.5, 'Color', cor1);
title('Resposta à Rampa','FontSize', 16);
xlabel('Tempo [s]', 'FontSize', 16);
ylabel('Posição [m]', 'FontSize', 16);
set(gca, 'FontSize', 16);
ylim([0 5]);
xlim([0 t(end)]);
grid on;
print('Figure_3_ramp', '-dpng', '-r300');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PLOTANDO OS POLOS E ZEROS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obter os polos e zeros
[p, z] = pzmap(G);

figure(4);
hold on;

% Plotar zeros (se houver)
if ~isempty(z)
    plot(real(z), imag(z), 'o', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Zeros','MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
end

% Plotar polos com marcador maior
if ~isempty(p)
    plot(real(p), imag(p), 'x', 'MarkerSize', 15, 'LineWidth', 2,'DisplayName', 'Poles', 'Color', 'r');
end

% Adicionar linhas de grade e eixos
grid on;
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';

% Grade secundária para melhor visualização
grid minor;
ax.GridAlpha = 0.3;
ax.MinorGridAlpha = 0.1;

% Definir limites dos eixos
xlim([-2, 2]);  % Ajuste conforme necessário
ylim([-1.5, 1.5]);  % Ajuste conforme necessário

% Adicionar rótulos
legend('Location', 'southeast', 'FontSize', 16);
xlabel('Eixo Real (σ)', 'FontSize', 16);
ylabel('Eixo Imaginário (jω)', 'FontSize', 16);
title('Mapa de Polos e Zeros', 'FontSize', 16);
set(gca, 'FontSize', 16);
hold off;
print('Figure_4_poles', '-dpng', '-r300');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%FINALIZANDO
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Operação realizada com sucesso
disp('Plotagem realizada com sucesso!')

%Deletando os arquivos .asv
delete *.asv