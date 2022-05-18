function cost=pid_cost_function(x,P,wc_des)

    Kp=x(1); Ki=x(2); Kd=x(3); Tf=x(4);
    
    s=tf('s');
    
    C=Kp+Ki/s+Kd*s/(Tf*s+1);
    
    L=P*C;
    margini=allmargin(L);
    
    if isempty(margini.PMFrequency)
        cost=100;
    else
        cost=(wc_des-margini.PMFrequency(end))^2;
    end

end