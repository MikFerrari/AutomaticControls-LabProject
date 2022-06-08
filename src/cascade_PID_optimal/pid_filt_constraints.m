function [cin,ceq]=pid_filt_constraints(x,P,w_vector,MS,wh,Fh_max,wl,Dl_max,PM_min)

% PID_FILT_CONTRAINTS Calcola i valori dei vincoli non lineari.
%   x = [Kp,Ki,Kd,Tfd,Tfu]: vettore dei parametri
%   P: modello continuo del processo
%   w_vector: pulsazioni in cui calcolare le funzioni di trasferimento

% cin<=0
% ceq==0
    
    ceq=[];
    
    Kp=x(1); Ki=x(2); Kd=x(3); Tfd=x(4); Tfu=x(5);
    
    s=tf('s');

    C=(Kp+Ki/s+Kd*s/(Tfd*s+1))*1/(Tfu*s+1);
    
    C_fr=freqresp(C,w_vector);  % frequency response
    P_fr=freqresp(P,w_vector);
    C_fr=C_fr(:);               % reshaping da 1x1xn a nx1
    P_fr=P_fr(:);               % reshaping da 1x1xn a nx1
    
    L_fr=C_fr.*P_fr;
    S_fr=1./(1+L_fr);   % S=1/(1+L)
    F_fr=L_fr.*S_fr;    % F=L/(1+L)=L*S
    DY_fr=P_fr.*S_fr;   % DY=Y/D=P/(1+L)=P*S

    cin(1)=max(abs(F_fr(w_vector>wh)))-Fh_max;
    cin(2)=max(abs(DY_fr(w_vector<wl)))-Dl_max;
    cin(3)=max(abs(S_fr))-MS;
    
    L=P*C;
    margini=allmargin(L);   % calcolo margini di L
    
    if isempty(margini.PMFrequency)    % se L ha margine di fase infinito
        cin(4)=0;
    else
        cin(4)=PM_min-min(margini.PhaseMargin);    % min(PM)>=PM_min -> PM_min-PM<=0
    end

end
