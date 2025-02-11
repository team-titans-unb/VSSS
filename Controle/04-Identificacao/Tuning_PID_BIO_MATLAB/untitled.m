raizesf = roots([41.7151 49.9998 49.9998]);   % Obtendo o valor das raices
absolutof = abs(raizesf);       % Valor absoluto das raices
mayorf = max(absolutof);        % Valor maximo das raices
e1if = 1/(mayorf*10);            % Valor do filtro para o Kp
unomenosalfaf = exp(-(deltaT/e1if))
alfaf = 1 - unomenosalfaf
