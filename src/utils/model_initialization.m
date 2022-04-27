% Definisco il sistema da controllare
% 1: system = scara da controllare
% 2: rigid_robot = modello che non considera l'elasticità
[system,rigid_robot]=getRoboticSystem('Echo');

% Mostro lo scara (sistema robotico da controllare)
% figure
% system.show;

% Uso la funzione compilata per il calcolo della dinamica
system.setForwardDynamics(@fdCodegen_win); 

% Tempo di campionamento
st=system.getSamplingPeriod;

% Creo il sistema controllato
cs=ControlledSystemScara(system,'Echo');
