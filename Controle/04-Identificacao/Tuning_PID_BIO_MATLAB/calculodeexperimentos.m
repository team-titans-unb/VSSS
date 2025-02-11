%% Limpar o workspace e o command Windows
clear all    % Limpa o Workspace
close all    % Limpa as janelas
clc          % Limpa o Command Window
%% parámetros de configuração do PSO
maxCycle = 100;                 % Número maximo de iterações
runtime = 5;                   % Número maximo de experimentos
GlobalMins = zeros(1,runtime); % Minimos Globales
S = 20;                        % Número de partículas
F = 1.25;                      % Factor de mutación (caso seja executado o algoritmo DE)
C = 0.75;                      % crosover rate (caso seja executado o algoritmo DE)
x_min = -5;                 % Limite minimo do espaço de busca
x_max = 5;                     % Limite maximo do espaço de busca
N = 3;                         % Número de dimenções (Kp, Ki, Kd)
ys = zeros(N);                 % Melhor global posições particulas
sp = 150;                       % SP utilizado para avaliar a função objetivo (em cm)
deltaT = 0.7;
%% Execução do algoritmo PSO

for r=1:runtime     % For utilizado para executar o total de experimentos
    tic;           % Inicia la medición del tiempo
    [GlobalMins(r) ys] = PSO(x_min,x_max,N,S,maxCycle,sp);  % Execução do algoritmo PSO
    %[GlobalMins(r) ys] = DE(x_min,x_max,S,N, maxCycle,sp,F,C);  % Execução do algoritmo DE
    %[GlobalMins(r) ys] = DE(x_min,x_max,S,N, maxCycle,sp,F,C);  % Execução do algoritmo DE
    %[GlobalMins(r) ys] = GOA(S, maxCycle,x_min,x_max,N,sp);  % Execução do algoritmo GOA
    %[GlobalMins(r) ys] = OGOA(S, maxCycle,x_min,x_max,N,sp);  % Execução do algoritmo OGOA
    %[GlobalMins(r) ys] = OMFO(S,maxCycle,x_min,x_max,N,sp) % Execução do algoritmo OMFO
    %[GlobalMins(r) ys] = MFO(S,maxCycle,x_min,x_max,N,sp)  % Execução do algoritmo MFO
    tiempo = toc;  % Detiene la medición y devuelve el tiempo transcurrido
    fprintf('El tiempo de ejecución fue: %.4f segundos\n', tiempo);
    kpi = ys(1);                    % O primeiro valor é o valor de Kp
    kii = ys(2);                    % O segundo valor é a constante Ki
    kdi = ys(3);                    % O terceiro valor é o valor da constante Kd
    raizes = roots([kdi kpi kii]);   % Obtendo o valor das raices
    %raizes = roots([kd kp ki]);
    absoluto = abs(raizes);       % Valor absoluto das raices
    mayor = max(absoluto);        % Valor maximo das raices
    e1i = 1/(mayor*10);            % Valor do filtro para o Kp
    unomenosalfa = exp(-(deltaT/e1i))
    alfa = 1 - unomenosalfa
    save([ 'PID_OptimizationPSO',num2str(r),'.mat']);  % Carrega os experimentos salvos
    disp('Esta no experimento:');                  % Apresenta Experimento
    disp(r); 
end                          % Finalização dos experiementos

melhorglobal = 100000000;       % Melhor globlal fitnes 
%% Para calcular o melhor dos experimentos 
 for r=1:runtime     %For para o número de experimentos


     load([ 'PID_OptimizationPSO',num2str(r),'.mat']);  % Carrega os experimentos salvos
     if (GlobalMins(r)<= melhorglobal)  % Caso o GlobalMin seja  menor ao anterio salva em "mejorglobal"
         melhorglobal=GlobalMins(r);  % Salva GlobalMin em "mejorglobal"
         kpfinal=ys(1);          % Salva o valor do kp melhor
         kifinal=ys(2);          % Salva o valor do ki melhor
         kdfinal=ys(3);          % Salva o valor do kd melhor
         raizesf = roots([kdfinal kpfinal kifinal]);   % Obtendo o valor das raices
         absolutof = abs(raizesf);       % Valor absoluto das raices
         mayorf = max(absolutof);        % Valor maximo das raices
         e1if = 1/(mayorf*10);            % Valor do filtro para o Kp
         unomenosalfaf = exp(-(deltaT/e1if))
         alfaf = 1 - unomenosalfaf
         disp('Melhor experimento foi');                  % Apresenta Experimento
         disp(r);                              % Apresenta o numero de experimento 
     end
         
 end 