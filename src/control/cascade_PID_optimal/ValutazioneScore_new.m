clear;
clc;
model_initialization;

% Accelerazione massima e Rest Time
cs.setMaxAcceleration(15);
cs.setRestTime(0.1);

% Loading dei controllori
load('.\src\controllers\params\PID_int_joint1.mat');
load('.\src\controllers\params\PID_ext_joint1.mat');
load('.\src\controllers\params\PID_int_joint2.mat');
load('.\src\controllers\params\PID_ext_joint2.mat');

%% Creazione sistema complessivo da simulare
controller_joint1 = CascadeController(st,PID_int_joint1.controller,PID_ext_joint1.controller);
controller_joint2 = CascadeController(st,PID_int_joint2.controller,PID_ext_joint2.controller);

controller = ScaraController(st,controller_joint1,controller_joint2,rigid_robot);

%% Simulazione e valutazione
% impostazione limiti di saturazione
umax = system.getUMax;
controller.setUMax(umax);

% setto il controllore
cs.setController(controller);

% inizializzazione controllore e sistema controllato
cs.initialize;

% calcolo dello score
n_tests = 1;
for i=1:n_tests
    cs.initialize;
    % simulo il sistema per la valutazione -> "fast" fa 5 esecuzioni || "complete" fa 5 x 5 esecuzioni
    [score,results] = cs.evalution("complete");
    fprintf('lo score è %f\n',score);
end

% grafico i risultati
warning off
% cs.showResults(results)
warning on