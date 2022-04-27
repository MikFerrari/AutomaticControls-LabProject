clear; clc;
Nprova = 16;
simulation.author = 'R';
startTimer = tic;

%% Setting modello e parametri
model_initialization;
w_max = pi/st;
umax = system.getUMax;

%% Setting segnale eccitante
% Chirp
w0_chirp = 1; w1_chirp = 3.1416e3;       % wide, main chirp
w0b_chirp = 10; w1b_chirp = 100;     % narrow, secondary chirp 1 
w0c_chirp = 50; w1c_chirp = 1000;     % narrow, secondary chirp 2
ampiezza_chirp = 65;

% Portante
w_portante = w0_chirp/20;
ampiezza_portante = 65;
T_portante = 2*pi/w_portante;

%% Costruzione del segnale eccitante

% Portante
durata_simulazione = 1.1*T_portante; % secondi
t = (0:st:durata_simulazione)';
portante = ampiezza_portante*sin(w_portante*t);

durata_teorica = seconds(durata_simulazione); % debug
durata_teorica.Format = 'hh:mm:ss';
display(durata_teorica)
n_samples = length(t); % debug
sample_computation_time = 0.0052; % stimato empiricamente su 100 run di cs.openloop (risulta circa 0.01, 10 volte il tempo di campionamento)
                                % => quindi la durata reale (quanto ci impiega il codice) sarà 10 volte la durata teorica calcolata sopra
durata_reale = n_samples*sample_computation_time; % debug
durata_reale = seconds(durata_reale); % debug
durata_reale.Format = 'hh:mm:ss';
display(durata_reale)

% Chirp
chirp_wide = chirp(t,w0_chirp/2/pi,t(end),w1_chirp/2/pi,'logarithmic',90); % chirp che spazza un range maggiore (90° per iniziare come un seno)
chirp_narrow1 = chirp(t,w0b_chirp/2/pi,t(end),w1b_chirp/2/pi,'logarithmic',135); % chirp concentrato attorno alla risonanza
chirp_narrow2 = chirp(t,w0c_chirp/2/pi,t(end),w1c_chirp/2/pi,'logarithmic',180); % chirp concentrato attorno al polo dell'attrito viscoso

chirp_total = (chirp_wide + chirp_narrow1 + chirp_narrow2)/max(chirp_wide + chirp_narrow1 + chirp_narrow2);
% chirp_total = chirp_wide;

chirpSignal = ampiezza_chirp*chirp_total;

% Somma dei segnali
input_signal = portante + chirpSignal;

fig_inputSignal = figure;
plot(t,input_signal);
xlabel('time'); ylabel('input - identification'); title('Segnale eccitante - Identificazione'); grid on;

%% Selezione dei sottosistemi di cui eseguire l'identificazione
joint1 = false;
joint2 = true;

if joint1 && ~ joint2
    control_action = [input_signal zeros(length(input_signal),1)];
elseif joint2 && ~ joint1
    control_action = [zeros(length(input_signal),1) input_signal];
elseif joint1 && joint2
    control_action = [input_signal input_signal];
else
    error('Selezionare almeno 1 giunto!')
end

%% Simulazione sistema
f = waitbar(0,'0','Name','Simulazione - Identificazione', 'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');
setappdata(f,'canceling',0);

execution_time = zeros(length(t),1); % debug

% Arrays initialization
process_output = NaN(length(t),4);
tt = NaN(length(t),1);

for idx=1:length(t)
    tic
    
    % Check for clicked Cancel button
    if getappdata(f,'canceling')
        break
    end
    
    [process_output(idx,:),tt(idx,1)]=cs.openloop(control_action(idx,:));

    % Update waitbar and message
    percent_progress = idx/length(t)*100;
    waitbar(percent_progress/100,f,sprintf('%.2f %%',percent_progress))

    execution_time(idx) = toc; % debug
end

beep % notification sound

delete(f)

disp('Tempo di esecuzione medio di una singola simulazione')
mean_execution_time = mean(execution_time) % debug

%% Calcolo risposta in frequenza
bode_opts = bodeoptions('cstprefs');
bode_opts.PhaseWrapping = 'on';

% Joint 1
if (joint1)
    identification_joint1 = iddata(process_output(:,3),control_action(1:length(tt),1),st);
    identification_joint1.OutputName = 'vel1'; identification_joint1.InputName = 'torque1';
    
    freq_resp_ident_joint1 = spafdr(identification_joint1);
    frequency = freq_resp_ident_joint1.Frequency;
    
    fig_j1 = figure;
    bode(freq_resp_ident_joint1,'-',bode_opts)
    grid on
    hold on
    plot(w0_chirp*[1 1],ylim,'-.r'); plot(w1_chirp*[1 1],ylim,'-.r')
    plot(w0b_chirp*[1 1],ylim,'--g'); plot(w1b_chirp*[1 1],ylim,'--g')
    plot(w0c_chirp*[1 1],ylim,'--k'); plot(w1c_chirp*[1 1],ylim,'--k')
    hold off
    title('F.d.t. coppia-velocità giunto 1')   
end

% Joint 2
if (joint2)
    identification_joint2 = iddata(process_output(:,4),control_action(1:length(tt),2),st);
    identification_joint2.OutputName = 'vel2'; identification_joint2.InputName = 'torque2';
    
    freq_resp_ident_joint2 = spafdr(identification_joint2);
    frequency = freq_resp_ident_joint2.Frequency;

    fig_j2 = figure;
    bode(freq_resp_ident_joint2,'-',bode_opts)
    grid on
    hold on
    plot(w0_chirp*[1 1],ylim,'-.r'); plot(w1_chirp*[1 1],ylim,'-.r')
    plot(w0b_chirp*[1 1],ylim,'--g'); plot(w1b_chirp*[1 1],ylim,'--g')
    plot(w0c_chirp*[1 1],ylim,'--k'); plot(w1c_chirp*[1 1],ylim,'--k')
    hold off
    title('F.d.t. coppia-velocità giunto 2')
end

%% Grafico dei risultati
pos1=process_output(:,1); pos2=process_output(:,2);
vel1=process_output(:,3); vel2=process_output(:,4);
tau1=control_action(:,1); tau2=control_action(:,2);

% coppia considerando la saturazione e solo i samples effettivamente
% calcolati durante la simulazione (anche se questa è stata interrotta a un certo punto)
actual_tau1 = min(max(tau1(1:length(tt)),-umax(1)),umax(1));
actual_tau2 = min(max(tau2(1:length(tt)),-umax(2)),umax(2));

fig_res = figure;
tl = tiledlayout(3,2);

nexttile; plot(tt,pos1); grid on; xlabel('t'); ylabel('pos 1'); ylim([1.1*min([pos1;pos2]) 1.1*max([pos1;pos2])])
nexttile; plot(tt,pos2); grid on; xlabel('t'); ylabel('pos 2'); ylim([1.1*min([pos1;pos2]) 1.1*max([pos1;pos2])])

nexttile; plot(tt,vel1); grid on; xlabel('t'); ylabel('vel 1'); ylim([1.1*min([vel1;vel2]) 1.1*max([vel1;vel2])])
nexttile; plot(tt,vel2); grid on; xlabel('t'); ylabel('vel 2'); ylim([1.1*min([vel1;vel2]) 1.1*max([vel1;vel2])])

nexttile; plot(tt,actual_tau1); grid on; xlabel('t'); ylabel('torque 1');
ylim([1.1*min([actual_tau1;actual_tau2]) 1.1*max([actual_tau1;actual_tau2])])

nexttile; plot(tt,actual_tau2); grid on; xlabel('t'); ylabel('torque 2');
ylim([1.1*min([actual_tau1;actual_tau2]) 1.1*max([actual_tau1;actual_tau2])])

title(tl,'Segnale eccitante e risposta del sistema')

%% Salvataggio simulazione
simulationName = strcat(simulation.author,'_simulation_',num2str(Nprova));
simulationDirectory = strcat('.\simulations\identification\',simulationName);

simulation.description = '3xchirp + portante - j1';
simulation.w0chirp = w0_chirp;
simulation.w1chirp = w1_chirp;
simulation.wportante = w_portante;
simulation.ampiezza_chirp = ampiezza_chirp;
simulation.ampiezza_portante = ampiezza_portante;
simulation.durata_simulazione = durata_simulazione;
simulation.control_action = control_action;
simulation.process_output = process_output;
simulation.tt = tt;
simulation.joint1 = joint1;
simulation.joint2 = joint2;
simulation.durata_teorica = durata_teorica;
simulation.durata_reale = durata_reale;
simulation.fig_j1 = fig_j1;
simulation.fig_j2 = fig_j2;
simulation.fig_res = fig_res;
simulation.fig_inputSignal = fig_inputSignal;

save(simulationDirectory,'-struct','simulation')