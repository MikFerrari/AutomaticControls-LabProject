function cost=pi_filt_cost_function(x,P)

    Kp=x(1); Ki=x(2);

    s=tf('s');
    
    C=(Kp+Ki/s);
    
    L=P*C;
    margini=allmargin(L);
    
    if isempty(margini.PMFrequency)
        cost=100;
    else
         cost=-margini.PMFrequency(end);
    end

end