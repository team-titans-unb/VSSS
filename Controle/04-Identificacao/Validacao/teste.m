clc; clear;

% Carregar os dados do arquivo CSV gerado anteriormente
filePath = 'Rampa.csv';
data = readmatrix(filePath);

% Definir o tempo de amostragem
Ts = 0.7; % segundos
numSamples = size(data, 1);
time = (0:numSamples-1) * Ts; % Vetor de tempo

% Extrair dados das colunas
pwm_right  = data(:,1) * 100 / 255; % PWM como set-point do motor direito em porcentagem
pwm_left   = data(:,2) * 100 / 255; % PWM como set-point do motor esquerdo em porcentagem
rpm_right  = data(:,3); % Velocidade medida pelo encoder direito
rpm_left   = data(:,4); % Velocidade medida pelo encoder esquerdo

%% Cálculo da média de velocidade para cada PWM, separadamente para cada motor

% PWM únicos para cada motor
pwm_unique_right = unique(pwm_right);
pwm_unique_left  = unique(pwm_left);

% Inicializar vetores de média
rpm_medio_right = zeros(size(pwm_unique_right));
rpm_medio_left  = zeros(size(pwm_unique_left));

% Média da velocidade para cada PWM no motor direito
for i = 1:length(pwm_unique_right)
    indices = (pwm_right == pwm_unique_right(i));
    rpm_medio_right(i) = mean(rpm_right(indices), 'omitnan');
end

% Média da velocidade para cada PWM no motor esquerdo
for i = 1:length(pwm_unique_left)
    indices = (pwm_left == pwm_unique_left(i));
    rpm_medio_left(i) = mean(rpm_left(indices), 'omitnan');
end

data_left = iddata(rpm_medio_left, pwm_unique_left, Ts);
data_right = iddata(rpm_medio_right, pwm_unique_left, Ts);
%% Gráficos

% Gráficos PWM vs. Velocidade Medida ao longo do tempo
figure;
subplot(2,1,1);
plot(time, pwm_right, 'r', 'LineWidth', 1.5); hold on;
plot(time, rpm_right, 'b', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Velocidade (RPM)');
title('Resposta do Motor Direito');
legend('Set-Point (PWM Right)', 'Velocidade Medida (Encoder Right)');
grid on;

subplot(2,1,2);
plot(time, pwm_left, 'r', 'LineWidth', 1.5); hold on;
plot(time, rpm_left, 'b', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Velocidade (RPM)');
title('Resposta do Motor Esquerdo');
legend('Set-Point (PWM Left)', 'Velocidade Medida (Encoder Left)');
grid on;

% Gráficos da relação PWM vs. Velocidade Média para cada motor
figure;
subplot(2,1,1);
plot(pwm_unique_right, rpm_medio_right, 'ko-', 'LineWidth', 1.5, 'MarkerFaceColor', 'k');
xlabel('PWM (%)'); ylabel('Velocidade Média (RPM)');
title('Relação PWM vs. Velocidade Média - Motor Direito');
grid on;

subplot(2,1,2);
plot(pwm_unique_left, rpm_medio_left, 'bo-', 'LineWidth', 1.5, 'MarkerFaceColor', 'b');
xlabel('PWM (%)'); ylabel('Velocidade Média (RPM)');
title('Relação PWM vs. Velocidade Média - Motor Esquerdo');
grid on;

%% Análise de desempenho

% Função para calcular parâmetros de desempenho
calcDesempenho = @(setpoint, resposta) struct( ...
    'Overshoot', 100 * (max(resposta) - setpoint(end)) / setpoint(end), ...
    'SettlingTime', find(abs(resposta - setpoint(end)) < 0.05 * setpoint(end), 1, 'last') * Ts, ...
    'Erro_Estado_Estacionario', 100 * (setpoint(end) - resposta(end)) / setpoint(end) ...
);

% Cálculo para o motor direito
desempenhoDireito = calcDesempenho(pwm_right, rpm_right);

% Cálculo para o motor esquerdo
desempenhoEsquerdo = calcDesempenho(pwm_left, rpm_left);

% Exibir resultados no console
fprintf('\nDesempenho do Motor Direito:\n');
fprintf('Overshoot: %.2f%%\n', desempenhoDireito.Overshoot);
fprintf('Settling Time: %.2f s\n', desempenhoDireito.SettlingTime);
fprintf('Erro em Estado Estacionário: %.2f%%\n', desempenhoDireito.Erro_Estado_Estacionario);

fprintf('\nDesempenho do Motor Esquerdo:\n');
fprintf('Overshoot: %.2f%%\n', desempenhoEsquerdo.Overshoot);
fprintf('Settling Time: %.2f s\n', desempenhoEsquerdo.SettlingTime);
fprintf('Erro em Estado Estacionário: %.2f%%\n', desempenhoEsquerdo.Erro_Estado_Estacionario);
