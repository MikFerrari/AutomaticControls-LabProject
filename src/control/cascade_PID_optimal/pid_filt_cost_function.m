function cost=pid_filt_cost_function(x,P,wc_des)

    Kp=x(1); Ki=x(2); Kd=x(3); Tfd=x(4); Tfu=x(5);
    
    s=tf('s');
    
    C=(Kp+Ki/s+Kd*s/(Tfd*s+1))*1/(Tfu*s+1);
    
    L=P*C;
    margini=allmargin(L);
    
    if isempty(margini.PMFrequency)
        cost=100;
    else
        cost=(wc_des-margini.PMFrequency(end))^2;
    end

end