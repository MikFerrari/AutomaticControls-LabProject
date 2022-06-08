function c=fourierCoefficients(t,y,omega0,omega)
% FOURIERCOEFFICIENTS Calcola i coefficienti di Fourier del segnale y.
%
%   t: asse dei tempi
%   y: segnale
%   omega0: pulsazione fondamentale
%   omega: vettore di pulsazioni del segnale

    for idx=1:length(omega)
        n=round(omega(idx)/omega0);                 % compute which multiple of omega0
        assert(abs(n-omega(idx)/omega0)<1e-3);      % check if omega is multiple of omega0
        c(idx,1)=fourierCoefficient(t,y,omega0,n);  % compute the single Fourier coefficient
    end
end