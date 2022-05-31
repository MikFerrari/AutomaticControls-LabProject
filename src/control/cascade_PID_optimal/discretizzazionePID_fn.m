function [num_contr_D, den_contr_D] = discretizzazionePID_fn(Kp,Ki,Kd,Tfd,Tfu,Ts)

s = tf('s');

% PID continuo in forma parallela
PID_C = tf(pid(Kp,Ki,Kd,Tfd));
PID_C_filt = PID_C * 1/(Tfu*s+1);

if Ki~=0
    % Scomposizione in fratti semplici
    [numC,denC] = tfdata(PID_C_filt); % calcolo numeratore e denomitore
    [res,poliC,costante] = residue(numC{1},denC{1}); % scomposizione in fratti semplici

    % Separazione controllori
    Ki = res(end);
    I_contr_C = Ki/s;                            % azione integrale
    PD_contr_C = minreal(PID_C_filt-I_contr_C);  % parte del controllore esclusa l'azione integrale
else
    PD_contr_C=PID_C_filt;
end
                                       
% Discretizzazione controllore PD_contr_C
PD_contr_D = c2d(PD_contr_C,Ts);
PD_contr_D.Variable='z^-1';

% Calcolo numeratore e denominatore
num_contr_D = PD_contr_D.Numerator{1};
den_contr_D = PD_contr_D.Denominator{1};

% Mi assicuro che il denominatore sia nella forma corretta dividendo per den_PD_contr_D(1)
num_contr_D = num_contr_D/den_contr_D(1);
den_contr_D = den_contr_D/den_contr_D(1);