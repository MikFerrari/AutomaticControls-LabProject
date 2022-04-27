function trans_func = fit_5order_model(dc_gain,M_Rp1_val,M_Rz1_val,M_Rp2_val,M_Rz2_val, ...
                                       w_h_val,w_Rp1_val,w_Rz1_val,w_Rp2_val,w_Rz2_val)
    
    syms s k w_h w_nz1 w_np1 xi_p1 xi_z1 w_Rp1 M_Rp1 w_Rz1 M_Rz1 ...
                 w_nz2 w_np2 xi_p2 xi_z2 w_Rp2 M_Rp2 w_Rz2 M_Rz2

    frf_sym = k*((s^2+2*xi_z1*w_nz1*s+w_nz1^2)*(s^2+2*xi_z2*w_nz2*s+w_nz2^2)) / ...
                ((s+w_h)*(s^2+2*xi_p1*w_np1*s+w_np1^2)*(s^2+2*xi_p2*w_np2*s+w_np2^2));

    % Valori di modulo e pulsazione alla risonanza
    w_Rp1 = w_np1*sqrt(1-2*xi_p1^2);
    w_Rp2 = w_np2*sqrt(1-2*xi_p2^2);
    w_Rz1 = w_nz1*sqrt(1-2*xi_z1^2);
    w_Rz2 = w_nz2*sqrt(1-2*xi_z2^2);
    
    M_Rp1 = 1/(2*xi_p1*sqrt(1-xi_p1^2));
    M_Rp2 = 1/(2*xi_p2*sqrt(1-xi_p2^2));
    M_Rz1 = 1/(2*xi_z1*sqrt(1-xi_z1^2));
    M_Rz2 = 1/(2*xi_z2*sqrt(1-xi_z2^2));

    % Valori letti da grafico e convertiti dalla scala logaritmica
    dc_gain = db2mag(dc_gain);
    M_Rp1_val = db2mag(M_Rp1_val);
    M_Rz1_val = db2mag(M_Rz1_val);
    M_Rp2_val = db2mag(M_Rp2_val);
    M_Rz2_val = db2mag(M_Rz2_val);
    
    % Calcolo di w_h w_nz1 xi_z1 w_np1 xi_p1 w_nz2 xi_z2 w_np2 xi_p2
    S_p1 = solve([M_Rp1 == M_Rp1_val, w_Rp1 == w_Rp1_val],[w_np1 xi_p1]);
    w_np1_val = real(double(S_p1.w_np1(1)));
    xi_p1_val = double(S_p1.xi_p1(1));

    S_p2 = solve([M_Rp2 == M_Rp2_val, w_Rp2 == w_Rp2_val],[w_np2 xi_p2]);
    w_np2_val = abs(double(S_p2.w_np2(1)));
    xi_p2_val = double(S_p2.xi_p2(1));
    
    S_z1 = solve([M_Rz1 == M_Rz1_val, w_Rz1 == w_Rz1_val],[w_nz1 xi_z1]);
    w_nz1_val = abs(double(S_z1.w_nz1(1)));
    xi_z1_val = double(S_z1.xi_z1(1));

    S_z2 = solve([M_Rz2 == M_Rz2_val, w_Rz2 == w_Rz2_val],[w_nz2 xi_z2]);
    w_nz2_val = abs(double(S_z2.w_nz2(1)));
    xi_z2_val = double(S_z2.xi_z2(1));
    
    % Calcolo di k
    k_val = dc_gain*w_h_val*w_np1_val^2*w_np2_val^2/(w_nz1_val^2*w_nz2_val^2);

    % Sostituzione valori calcolati
    frf_val = subs(frf_sym,{'k','w_h','w_nz1','w_np1','w_nz2','w_np2', ...
                            'xi_z1','xi_p1','xi_z2','xi_p2'}, ...
                           {k_val,w_h_val,w_nz1_val,w_np1_val,w_nz2_val,w_np2_val, ...
                            xi_z1_val,xi_p1_val,xi_z2_val,xi_p2_val});

    % Calcolo funzione di trasferimento fattorizzata
    [num,den] = numden(frf_val);
    num = sym2poly(expand(num));
    den = sym2poly(expand(den));
    trans_func = minreal(tf(num,den));
    trans_func = zpk(trans_func);

end
