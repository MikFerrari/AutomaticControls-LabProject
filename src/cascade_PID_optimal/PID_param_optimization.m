function x = PID_param_optimization(aKp,aKi,aKd,aTfd,aTfu,J,nlcon,timeout,optimizer)
% PID_PARAM_OPTIMIZATION Ottimizza i parametri di un PID filtrato.
%
%   - aKp,aKi,aKd,aTfd,aTfu contengono ciascuno il valore iniziale, il mimite
%       inferiore e il limite superiore che può assumere la variabile.
%   - nlcon: vincoli non lineari
%   - optimizer: ottimizzatore scelto

    x0 = [aKp(1), aKi(1), aKd(1), aTfd(1), aTfu(1)];
    
    problem = createOptimProblem('fmincon','x0',x0, ...
                                 'objective',J, ...
                                 'lb',[aKp(2), aKi(2), aKd(2), aTfd(2), aTfu(2)], ...
                                 'ub',[aKp(3), aKi(3), aKd(3), aTfd(3), aTfu(3)], ...
                                 'nonlcon',nlcon);
    
    if strcmp(optimizer,'GlobalSearch')  % GlobalSearch non supporta il calcolo parallelo
        solver = GlobalSearch('Display','iter','MaxTime',timeout);
        x = run(solver,problem)
    else % strcmp(optimizer,'MultiStart')
        solver = MultiStart('Display','iter','MaxTime',timeout,'UseParallel',true);
        addAttachedFiles(gcp,["pid_filt_constraints.m" "pid_filt_cost_function.m"]) % Usare se si abilita il pool parallelo su un Cluster online
        n_startingPoints = 50;
        x = run(solver,problem,n_startingPoints)
    end

end