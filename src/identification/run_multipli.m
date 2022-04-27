clear; clc; close all;
i=1;
w0_chirp = 1;

% Definizione parametri di simulazione
P1.ampiezza_chirp = 120;
P1.ampiezza_portante = 10;
P1.w_portante = w0_chirp/20;
P1.T_portante = 2*pi/P1.w_portante;
P1.durata_simulazione = 2*P1.T_portante;


P2.ampiezza_chirp = 120;
P2.ampiezza_portante = 20;
P2.w_portante = w0_chirp/20;
P2.T_portante = 2*pi/P2.w_portante;
P2.durata_simulazione = 2*P2.T_portante;

P3.ampiezza_chirp = 100;
P3.ampiezza_portante = 40;
P3.w_portante = w0_chirp/20;
P3.T_portante = 2*pi/P3.w_portante;
P3.durata_simulazione = 2*P3.T_portante;

P4.ampiezza_chirp = 120;
P4.ampiezza_portante = 20;
P4.w_portante = w0_chirp/10;
P4.T_portante = 2*pi/P4.w_portante;
P4.durata_simulazione = 2*P4.T_portante;

P5.ampiezza_chirp = 100;
P5.ampiezza_portante = 40;
P5.w_portante = w0_chirp/10;
P5.T_portante = 2*pi/P5.w_portante;
P5.durata_simulazione = 2*P5.T_portante;


P = {P1; P2; P3; P4; P5};
for Nprova = 6:10

    % Scelta dei parametri di simulazione
    p = P{i};
    ampiezza_chirp = p.ampiezza_chirp;
    ampiezza_portante = p.ampiezza_portante;
    durata_simulazione = p.durata_simulazione;
    w_portante = p.w_portante;
    T_portante = p.T_portante;
    
    identificazione_proveRicky;
    i = i+1;
end