function c=fourierCoefficient(t,y,omega0,n)
% FOURIERCOEFFICIENT Calcola l'n-esimo coefficiente di Fourier del segnale y.
%
%   t: asse dei tempi
%   y: segnale
%   omega0: pulsazione fondamentale
%   n: indice del coefficiente

    T=2*pi/omega0;
    periods=floor(t(end)/T);
    assert(periods>=1)
    
    % Considerare solo gli ultimi periods*T secondi (evitare influenza delle condizioni iniziali)
    idx=find(t>=(t(end)-periods*T),1);
    t=t(idx:end);
    y=y(idx:end);
    
    % Dalla definizione della serie di Fourier, mediando su più periodi
    c=trapz(t,y.*exp(-1i*n*omega0*t))/T/periods;

end