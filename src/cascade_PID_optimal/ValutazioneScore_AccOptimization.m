clear; clc; close all;
n_prove = 3; % numero di prove per ciascuna accelerazione

% inizializzazione del modello
model_initialization;

% loading dei controllori
load('.\src\controllers\params\PID_int_joint1.mat');
load('.\src\controllers\params\PID_ext_joint1.mat');
load('.\src\controllers\params\PID_int_joint2.mat');
load('.\src\controllers\params\PID_ext_joint2.mat');

%% Creazione sistema complessivo da simulare
% due controllori in cascata
controller_joint1 = CascadeController(st,PID_int_joint1.controller,PID_ext_joint1.controller);
controller_joint2 = CascadeController(st,PID_int_joint2.controller,PID_ext_joint2.controller);

% controllore scara complessivo
controller = ScaraController(st,controller_joint1,controller_joint2,rigid_robot);

%% Inizializzazione
for i=15:1:30
    score_sum = 0;
    for j=1:n_prove

        % accelerazione massima e Rest Time
        cs.setMaxAcceleration(i);
        cs.setRestTime(0.1);
        
        % impostazione limiti di saturazione
        umax = system.getUMax;
        controller.setUMax(umax);
        
        % setto il controllore
        cs.setController(controller);
        
        % inizializzazione controllore e sistema controllato
        cs.initialize;
        
        % Simulazione e calcolo dello score
        score = cs.evalution("complete");
        score_sum = score_sum + score;
    end
    simulations(i-14,1) = i;
    simulations(i-14,2) = score_sum/n_prove;
end
simulations = array2table(simulations);
simulations.Properties.VariableNames = ["acc_max","punteggio"];