function percent_fit = compute_fit_goodness(data,estimate,frequency,lower_bound,upper_bound)
    
    measured = data(frequency > lower_bound & frequency < upper_bound);
    estimated = estimate(frequency > lower_bound & frequency < upper_bound);

    percent_fit = 100*(1-norm(measured-estimated)/norm(measured-mean(measured)));

end

