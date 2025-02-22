% Limpar workspace e fechar figuras
clear;
clc;
close all;

% Definir a planta (insira sua função de transferência da planta)
planta = tf(4.5517, [2.75 1]);
controlador1 = tf([0.8302, 0.1508], [1 0]);
controlador2 = tf([0.8394, 0.1528], [1 0]);

% Função de transferência em malha aberta para cada controlador
malha_aberta1 = controlador1 * planta;
malha_aberta2 = controlador2 * planta;

% Plotar o Root Locus para cada sistema controlado
figure;
rlocus(malha_aberta1);
title('LGR - Motor Esquerdo');
grid on;

figure;
rlocus(malha_aberta2);
title('LGR - Motor Direito');
grid on;

% Função de transferência em malha fechada com ganho 1
malha_fechada1 = feedback(malha_aberta1, 1);
malha_fechada2 = feedback(malha_aberta2, 1);

% Obter os polos (raízes) da função de transferência em malha fechada
polos_malha_fechada1 = pole(malha_fechada1);
polos_malha_fechada2 = pole(malha_fechada2);

% Exibir os polos
disp('Polos da malha fechada motor Direito:');
disp(polos_malha_fechada1);

disp('Polos da malha fechada motor Esquerdo:');
disp(polos_malha_fechada2);