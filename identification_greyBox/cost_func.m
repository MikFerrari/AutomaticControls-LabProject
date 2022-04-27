function cost = cost_func(x,tvec,qsp,qsp_d,Kp,Kv,system,system_estimate)
    
    system_estimate.initialize;

    system_estimate.hm(1,1)=x(1);
    system_estimate.hm(2,2)=x(2);
    system_estimate.cm(1)=x(3);
    system_estimate.cm(2)=x(4);
    system_estimate.k(1,1)=x(5);
    system_estimate.k(2,2)=x(6);
    system_estimate.h(1,1)=x(7);
    system_estimate.h(2,2)=x(8);
    system_estimate.JmInv(1,1)=x(9)^-1;
    system_estimate.JmInv(2,2)=x(10)^-1;

    q_est=zeros(size(qsp,1),size(qsp,2));
    qd_est=zeros(size(qsp,1),size(qsp,2));
    tau=zeros(size(qsp,1),size(qsp,2));
    q_est(:,1)=qsp(:,1);
    
    q=zeros(size(qsp,1),size(qsp,2));
    qd=zeros(size(qsp,1),size(qsp,2));
    
    for idx=1:length(tvec)-1
        y_est=system_estimate.computeOutput;
        y=system.computeOutput;
        q_est(:,idx)=y_est(1:2);
        qd_est(:,idx)=y_est(3:4);
    
        q(:,idx)=y(1:2);
        qd(:,idx)=y(3:4);
    
        tau(:,idx)=Kp*(qsp(:,idx)-q_est(:,idx))+Kv*(qsp_d(:,idx)-qd_est(:,idx));
        system_estimate.updateState(tau(:,idx),tvec(idx));
        system.updateState(tau(:,idx),tvec(idx));
    end
    
    cost=sum((q_est(1,:)-q(1,:)).^2);
end

