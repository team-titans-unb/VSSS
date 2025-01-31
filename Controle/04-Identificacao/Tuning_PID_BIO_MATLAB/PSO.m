%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Algoritmo para fazer a sintonização do controlador PID utilizando PSO
%%Mario Andrés Pastrana Triana
%%Estudante de mestrado da UnB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [out, ys]=PSO (x_min,x_max,N,S,maxiter,sp)
%% Configurações do PSO
w0 = 0.9;                   % Fator de inercia inicial
wf = 0.1;                   % Fator de inercia final
w = w0;                     % Declaração da inercia
slope = (w0-wf)/maxiter;    % Pasos da inercia
c1 = 2.05;                  % Coeficiente cognitivo
c2 = 2.05;                  % Coeficiente social
max_v =(x_max-x_min)*2 ;    % Valor maximo de velocidade
ini_v = max_v/3;            % Velocidade inicial 
k = 1;                      % Contador de iterações

%% Inicialização do PSO
x = zeros(S,N);             % Criação das S particulas nas N dimensões
f_ind = 1e10*ones(S,1);     % Inicializando o melhor resultado da função de cada particula

for j=1:N                   % For das dimensões
    for i=1:S               % For das particulas
        x(i,j) = x_min + (x_max-x_min) * rand(); % Inicializando as particulas em posições aleatorias
        y(i,j) = 1e10;                           % Melhor de cada particulas
        v(i,j) = ini_v;                          % Velocidade inicial
    end
end

%% Processo Iterativo

for k=1: maxiter    % ´Executa o número de iterações
    %%  Melhor Local de cada particula
    
    for i = 1:S     % For para avaliar cada particula na função objetivo
                    % Calcula o melhor de cada particula na função objetivo
        fx(i) = tracklsq(x(i,:),sp);% Avalia cada particula na função objetivo
        if fx(i) < f_ind(i)         % Quando a avaliação da função da particula é menor ao valor global dela  
            y(i,:) = x(i,:);        % Salva o valor da posição da particula em y
            f_ind(i) = fx(i);       % Atualiza a o melhor resultado da particula
        end
    end
    
    %% Melhor Global (Melhor resultado do exame)
    
    [bestfitness(k), p] = min(f_ind); % Faz a deteção do melhor valor de particula do examen
    ys = y(p,:);                      % Salva a posição da melor particula em ys
    
    %% Atualização da posição e velocidade das particulas

    for j = 1:N   % For das dimensões         
        for i=1:S % For das particulas
            r1 = rand();    % Nùmero aleatoria para a ecuação do PSO
            r2 = rand();    % Nùmero aleatoria para a ecuação do PSO
            
            v(i,j) = w*v(i,j) + c1*r1*(y(i,j)-x(i,j)) + c2*r2*(ys(j) - x(i,j)); % Equação da velocidade do PSO
            
            if abs(v(i,j)) > max_v  % Limite da velocidade de cada particulas em cada uma das dimensões
                if v(i,j) > 0       % Quando a velociade é maior a 0
                    v(i,j) = max_v; % O valor da particula é o valor maximo (limita a este valor)
                else                % Quando é menor a 0
                    v(i,j) = -max_v;% O valor da particula é -ValorMaximo (limita a velocidade)
                end
            end
            
            x(i,j) = x(i,j) + v(i,j); % Atualização da posição de cada particula
%                    
            if(x(i,j) > x_max)                        % Quando a posição atual é maior ao espaço de busca
                x(i,j) = x_max-(abs(x_min)*(rand())); % Coloca a particula numa posição aleatoria (dentro do espaço de busca)
            elseif (x(i,j) < x_min)                   % Quando a posição atual é menor ao espaço de busca
                x(i,j) = x_min+(x_max*(rand()));      % Coloca a particula numa posição aleatoria (dentro do espaço de busca)
            end
        end
    end    

    w = w - slope;  % A inercia decrece a cada iteração
    disp(k)         % Apresenta em que iteração esta o algoritmo
end

plot(bestfitness);      % Grafica a curva de convergencia do algoritmo PSO
out = bestfitness(k-1); % Salva em out o melhor resultado da avaliação
disp (out)              % Apresenta o melhor resultado da função objetivo
disp(ys)                % Apresenta a posição do melhor resultado da função objetivo
               
end