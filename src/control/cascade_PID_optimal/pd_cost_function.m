function cost=pd_cost_function(x,P,wc_des)

    Kp=x(1); Kd=x(2); Tf=x(3);
    
    s=tf('s');
    
    C=Kp+Kd*s/(Tf*s+1);
    
    L=P*C;
    margini=allmargin(L);
    
    if isempty(margini.PMFrequency)
        cost=100;
    else
        cost=(wc_des-margini.PMFrequency(end))^2;
    end

end