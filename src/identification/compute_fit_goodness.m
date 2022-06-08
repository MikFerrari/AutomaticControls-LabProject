function percent_fit=compute_fit_goodness(data,estimate,frequency,lower_bound,upper_bound)
% COMPUTE_FIT_GOODNESS Calcola la percentuale di fitness in un certo intervallo.
% 
%   La percentuale di fitness è calcolata con la formula (NRMSE):
%   percent_fit=100*(1-norm(measured-estimated)/norm(measured-mean(measured)));

    measured=data(frequency>lower_bound & frequency<upper_bound);
    estimated=estimate(frequency>lower_bound & frequency<upper_bound);

    percent_fit=100*(1-norm(measured-estimated)/norm(measured-mean(measured)));
end

