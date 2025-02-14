clc; clear;

% Carregar os dados do arquivo CSV gerado anteriormente
filePath = 'validacao_2.csv';
data = readmatrix(filePath);

% Definir o tempo de amostragem
Ts = 0.7; % segundos
numSamples = size(data, 1);
time = (0:numSamples-1) * Ts; % Vetor de tempo

% Extrair dados das colunas
SetPoint_Right  = data(:,1); % PWM como set-point do motor direito
SetPoint_Left   = data(:,2); % PWM como set-point do motor esquerdo
RPM_Right_meas  = data(:,3); % Velocidade medida pelo encoder direito
RPM_Left_meas   = data(:,4); % Velocidade medida pelo encoder esquerdo

% Criar a figura com dois subgráficos
figure;
subplot(2,1,1);
plot(time, SetPoint_Right, 'r--', 'LineWidth', 1.5); hold on;
plot(time, RPM_Right_meas, 'b', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Velocidade (RPM)');
title('Resposta do Motor Direito');
legend('Set-Point (PWM Right)', 'Velocidade Medida (Encoder Right)');
grid on;

subplot(2,1,2);
plot(time, SetPoint_Left, 'r--', 'LineWidth', 1.5); hold on;
plot(time, RPM_Left_meas, 'b', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Velocidade (RPM)');
title('Resposta do Motor Esquerdo');
legend('Set-Point (PWM Left)', 'Velocidade Medida (Encoder Left)');
grid on;

%% Análise de desempenho

% Função para calcular parâmetros de desempenho
calcDesempenho = @(setpoint, resposta) struct( ...
    'Overshoot', 100 * (max(resposta) - setpoint(end)) / setpoint(end), ...
    'SettlingTime', find(abs(resposta - setpoint(end)) < 0.05 * setpoint(end), 1, 'last') * Ts, ...
    'Erro_Estado_Estacionario', 100 * (setpoint(end) - resposta(end)) / setpoint(end) ...
);

% Cálculo para o motor direito
desempenhoDireito = calcDesempenho(SetPoint_Right, RPM_Right_meas);

% Cálculo para o motor esquerdo
desempenhoEsquerdo = calcDesempenho(SetPoint_Left, RPM_Left_meas);

% Exibir resultados no console
fprintf('\nDesempenho do Motor Direito:\n');
fprintf('Overshoot: %.2f%%\n', desempenhoDireito.Overshoot);
fprintf('Settling Time: %.2f s\n', desempenhoDireito.SettlingTime);
fprintf('Erro em Estado Estacionário: %.2f%%\n', desempenhoDireito.Erro_Estado_Estacionario);

fprintf('\nDesempenho do Motor Esquerdo:\n');
fprintf('Overshoot: %.2f%%\n', desempenhoEsquerdo.Overshoot);
fprintf('Settling Time: %.2f s\n', desempenhoEsquerdo.SettlingTime);
fprintf('Erro em Estado Estacionário: %.2f%%\n', desempenhoEsquerdo.Erro_Estado_Estacionario);
