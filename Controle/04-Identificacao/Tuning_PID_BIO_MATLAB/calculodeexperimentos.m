%% Limpar o workspace e o command Windows
clear all    % Limpa o Workspace
close all    % Limpa as janelas
clc          % Limpa o Command Window
%% par�metros de configura��o do PSO
maxCycle = 100;                 % N�mero maximo de itera��es
runtime = 5;                   % N�mero maximo de experimentos
GlobalMins = zeros(1,runtime); % Minimos Globales
S = 20;                        % N�mero de part�culas
F = 1.25;                      % Factor de mutaci�n (caso seja executado o algoritmo DE)
C = 0.75;                      % crosover rate (caso seja executado o algoritmo DE)
x_min = -5;                 % Limite minimo do espa�o de busca
x_max = 5;                     % Limite maximo do espa�o de busca
N = 3;                         % N�mero de dimen��es (Kp, Ki, Kd)
ys = zeros(N);                 % Melhor global posi��es particulas
sp = 150;                       % SP utilizado para avaliar a fun��o objetivo (em cm)
deltaT = 0.7;
%% Execu��o do algoritmo PSO

for r=1:runtime     % For utilizado para executar o total de experimentos
    tic;           % Inicia la medici�n del tiempo
    [GlobalMins(r) ys] = PSO(x_min,x_max,N,S,maxCycle,sp);  % Execu��o do algoritmo PSO
    %[GlobalMins(r) ys] = DE(x_min,x_max,S,N, maxCycle,sp,F,C);  % Execu��o do algoritmo DE
    %[GlobalMins(r) ys] = DE(x_min,x_max,S,N, maxCycle,sp,F,C);  % Execu��o do algoritmo DE
    %[GlobalMins(r) ys] = GOA(S, maxCycle,x_min,x_max,N,sp);  % Execu��o do algoritmo GOA
    %[GlobalMins(r) ys] = OGOA(S, maxCycle,x_min,x_max,N,sp);  % Execu��o do algoritmo OGOA
    %[GlobalMins(r) ys] = OMFO(S,maxCycle,x_min,x_max,N,sp) % Execu��o do algoritmo OMFO
    %[GlobalMins(r) ys] = MFO(S,maxCycle,x_min,x_max,N,sp)  % Execu��o do algoritmo MFO
    tiempo = toc;  % Detiene la medici�n y devuelve el tiempo transcurrido
    fprintf('El tiempo de ejecuci�n fue: %.4f segundos\n', tiempo);
    kpi = ys(1);                    % O primeiro valor � o valor de Kp
    kii = ys(2);                    % O segundo valor � a constante Ki
    kdi = ys(3);                    % O terceiro valor � o valor da constante Kd
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
end                          % Finaliza��o dos experiementos

melhorglobal = 100000000;       % Melhor globlal fitnes 
%% Para calcular o melhor dos experimentos 
 for r=1:runtime     %For para o n�mero de experimentos


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