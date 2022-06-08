% Run degli script di tuning e di valutazione
clear; clc;

matlab.internal.liveeditor.executeAndSave(which('.\src\control\cascade_PID_optimal\TuningPID_joint1.mlx'));
matlab.internal.liveeditor.executeAndSave(which('.\src\control\cascade_PID_optimal\TuningPID_joint2.mlx'));
ValutazioneScore_new;