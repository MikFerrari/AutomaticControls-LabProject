function trans_func = fit_3order_model(dc_gain,M_Rp_val,M_Rz_val,w_h_val,w_Rp_val,w_Rz_val)
    
    syms s k w_h w_nz w_np xi_p xi_z w_Rp M_Rp w_Rz M_Rz

    frf_sym = k*(s^2+2*xi_z*w_nz*s+w_nz^2)/((s+w_h)*(s^2+2*xi_p*w_np*s+w_np^2));

    w_Rp = w_np*sqrt(1-2*xi_p^2);
    w_Rz = w_nz*sqrt(1-2*xi_z^2);
    
    M_Rp = 1/(2*xi_p*sqrt(1-xi_p^2));
    M_Rz = 1/(2*xi_z*sqrt(1-xi_z^2));
    
    % Valori letti da grafico e convertiti dalla scala logaritmica
    dc_gain = db2mag(dc_gain);
    M_Rp_val = db2mag(M_Rp_val);
    M_Rz_val = db2mag(M_Rz_val);
    
    % Calcolo di w_h w_nz w_np xi_p xi_z
    S_p = solve([M_Rp == M_Rp_val, w_Rp == w_Rp_val],[w_np xi_p]);
    w_np_val = abs(double(S_p.w_np(1)));
    xi_p_val = double(S_p.xi_p(1));
    
    S_z = solve([M_Rz == M_Rz_val, w_Rz == w_Rz_val],[w_nz xi_z]);
    w_nz_val = abs(double(S_z.w_nz(1)));
    xi_z_val = double(S_z.xi_z(1));
    
    % Calcolo di k
    k_val = dc_gain*w_h_val*w_np_val^2/w_nz_val^2;

    % Sostituzione valori calcolati
    frf_val = subs(frf_sym,{'k','w_h','w_nz','w_np','xi_z','xi_p'}, ...
                           {k_val,w_h_val,w_nz_val,w_np_val,xi_z_val,xi_p_val});

    % Calcolo funzione di trasferimento fattorizzata
    [num,den] = numden(frf_val);
    num = sym2poly(expand(num));
    den = sym2poly(expand(den));
    trans_func = minreal(tf(num,den));
    trans_func = zpk(trans_func);

end

