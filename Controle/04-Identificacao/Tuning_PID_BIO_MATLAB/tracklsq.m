%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Fun��o utilizada para realizar a sintoniza��o do controlador PID
%%Mario Andr�s Pastrana Triana
%%Estudante de mestrado da UnB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function F = tracklsq(pid,sp)
         kp = pid(1); % A primeira posi��o do pid � kp
         ki = pid(2); % A segunda posi��o do pid � ki
         kd = pid(3); % A terceira posi��o do pid � kd
         raizes=roots([kd kp ki]); % Encontra as raices do controlador PID
         absoluto=abs(raizes);     % Encontra o valor absoluto das raices
         mayor=max(absoluto);      % Encntra a raiz maior
         e1=1/(mayor*10);          % Encontra o valor do filto do PID
         simopt = simset('solver','ode23','SrcWorkspace','Current','DstWorkspace','Current');  % Inicializa as op��es do simulador
         [tout,xout,yout] = sim('Tunning_Bioinspired_PID',[0 60],simopt);   % Inicializa a simula��o com o arquivo optsim1_new
         overshot=abs(sp - max(yout));
         ua = numel(yout);
         erroS=abs(yout(ua)-sp);   % Calcula el error en estado estado estacionario
         Errortempo=abs(sp-yout(round(ua/100)));             % Erro absoluto acumulado  
         F=Errortempo*0.3+erroS*0.4 + overshot*0.3;    % Fun��o objetivo

    end