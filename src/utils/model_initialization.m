%% Generazione processo simulato e sistema controllato

% Definizione sistema da controllare
% system = SCARA da controllare
% rigid_robot = modello dello SCARA che non considera l'elasticità
[system,rigid_robot]=getRoboticSystem('Echo');

% Visualizzazione del robot SCARA
% figure
% system.show;

% Utilizzo della funzione compilata per calcolo della dinamica diretta
system.setForwardDynamics(@fdCodegen_win); 

% Tempo di campionamento
st=system.getSamplingPeriod;

% Creo il sistema controllato
cs=ControlledSystemScara(system,'Echo');
