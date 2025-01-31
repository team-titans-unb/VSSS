%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Algoritmo para fazer a sintoniza��o do controlador PID utilizando PSO
%%Mario Andr�s Pastrana Triana
%%Estudante de mestrado da UnB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [out, ys]=PSO (x_min,x_max,N,S,maxiter,sp)
%% Configura��es do PSO
w0 = 0.9;                   % Fator de inercia inicial
wf = 0.1;                   % Fator de inercia final
w = w0;                     % Declara��o da inercia
slope = (w0-wf)/maxiter;    % Pasos da inercia
c1 = 2.05;                  % Coeficiente cognitivo
c2 = 2.05;                  % Coeficiente social
max_v =(x_max-x_min)*2 ;    % Valor maximo de velocidade
ini_v = max_v/3;            % Velocidade inicial 
k = 1;                      % Contador de itera��es

%% Inicializa��o do PSO
x = zeros(S,N);             % Cria��o das S particulas nas N dimens�es
f_ind = 1e10*ones(S,1);     % Inicializando o melhor resultado da fun��o de cada particula

for j=1:N                   % For das dimens�es
    for i=1:S               % For das particulas
        x(i,j) = x_min + (x_max-x_min) * rand(); % Inicializando as particulas em posi��es aleatorias
        y(i,j) = 1e10;                           % Melhor de cada particulas
        v(i,j) = ini_v;                          % Velocidade inicial
    end
end

%% Processo Iterativo

for k=1: maxiter    % �Executa o n�mero de itera��es
    %%  Melhor Local de cada particula
    
    for i = 1:S     % For para avaliar cada particula na fun��o objetivo
                    % Calcula o melhor de cada particula na fun��o objetivo
        fx(i) = tracklsq(x(i,:),sp);% Avalia cada particula na fun��o objetivo
        if fx(i) < f_ind(i)         % Quando a avalia��o da fun��o da particula � menor ao valor global dela  
            y(i,:) = x(i,:);        % Salva o valor da posi��o da particula em y
            f_ind(i) = fx(i);       % Atualiza a o melhor resultado da particula
        end
    end
    
    %% Melhor Global (Melhor resultado do exame)
    
    [bestfitness(k), p] = min(f_ind); % Faz a dete��o do melhor valor de particula do examen
    ys = y(p,:);                      % Salva a posi��o da melor particula em ys
    
    %% Atualiza��o da posi��o e velocidade das particulas

    for j = 1:N   % For das dimens�es         
        for i=1:S % For das particulas
            r1 = rand();    % N�mero aleatoria para a ecua��o do PSO
            r2 = rand();    % N�mero aleatoria para a ecua��o do PSO
            
            v(i,j) = w*v(i,j) + c1*r1*(y(i,j)-x(i,j)) + c2*r2*(ys(j) - x(i,j)); % Equa��o da velocidade do PSO
            
            if abs(v(i,j)) > max_v  % Limite da velocidade de cada particulas em cada uma das dimens�es
                if v(i,j) > 0       % Quando a velociade � maior a 0
                    v(i,j) = max_v; % O valor da particula � o valor maximo (limita a este valor)
                else                % Quando � menor a 0
                    v(i,j) = -max_v;% O valor da particula � -ValorMaximo (limita a velocidade)
                end
            end
            
            x(i,j) = x(i,j) + v(i,j); % Atualiza��o da posi��o de cada particula
%                    
            if(x(i,j) > x_max)                        % Quando a posi��o atual � maior ao espa�o de busca
                x(i,j) = x_max-(abs(x_min)*(rand())); % Coloca a particula numa posi��o aleatoria (dentro do espa�o de busca)
            elseif (x(i,j) < x_min)                   % Quando a posi��o atual � menor ao espa�o de busca
                x(i,j) = x_min+(x_max*(rand()));      % Coloca a particula numa posi��o aleatoria (dentro do espa�o de busca)
            end
        end
    end    

    w = w - slope;  % A inercia decrece a cada itera��o
    disp(k)         % Apresenta em que itera��o esta o algoritmo
end

plot(bestfitness);      % Grafica a curva de convergencia do algoritmo PSO
out = bestfitness(k-1); % Salva em out o melhor resultado da avalia��o
disp (out)              % Apresenta o melhor resultado da fun��o objetivo
disp(ys)                % Apresenta a posi��o do melhor resultado da fun��o objetivo
               
end