clear; clc; close all;

% inizializzazione del modello
model_initialization;

% accelerazione massima e Rest Time
cs.setMaxAcceleration(15);
cs.setRestTime(0.1);

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
% impostazione limiti di saturazione
umax = system.getUMax;
controller.setUMax(umax);

% setto il controllore
cs.setController(controller);

% inizializzazione controllore e sistema controllato
cs.initialize;

%% Simulazione e calcolo dello score
% simulo il sistema per la valutazione -> "fast" fa 5 esecuzioni || "complete" fa 5 x 5 esecuzioni
[score,results] = cs.evalution("complete");
fprintf('lo score è %f\n',score);

% grafico i risultati
warning off
% cs.showResults(results)
warning on