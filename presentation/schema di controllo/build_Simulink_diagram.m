% Script per inizializzare lo schema simulink senza indicazioni di errore
% (funzione solo estetica, valori casuali)

clear; clc; close all;
s = tf('s');
Joint=1/(s+1);
Cv=1/s;
Cp=1/s;
Kp_p = 1;
Ki_p = 1;
Kp_v = 1;
Tfu = 1/70;
Kaw=Ki_p;