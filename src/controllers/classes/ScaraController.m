classdef ScaraController < BaseController
    
    properties  (Access = protected)
        ctrl_joint1 % cascata giunto 1
        ctrl_joint2 % cascata giunto 2
        model       % modello del sistema
    end

    methods

        function obj=ScaraController(st,cascade1,cascade2,model)
            obj@BaseController(st);
            obj.ctrl_joint1=cascade1;
            obj.ctrl_joint2=cascade2;
            obj.model=model;
        end

        function setUMax(obj,umax)
            % setto limiti di coppia controllori giunti
            setUMax@BaseController(obj,umax);
            obj.ctrl_joint1.setUMax(obj.umax(1));
            obj.ctrl_joint2.setUMax(obj.umax(2));
        end

        function initialize(obj)
            %inizializzo controllori
            obj.ctrl_joint1.initialize;
            obj.ctrl_joint2.initialize;
        end

        function u=computeControlAction(obj,reference,y)
            
            % setpoint
            sp_pos1=reference(1);
            sp_pos2=reference(2);
            sp_vel1=reference(3);
            sp_vel2=reference(4);
            sp_acc1=reference(5);
            sp_acc2=reference(6);

            % misure
            pos1=y(1);
            pos2=y(2);
            vel1=y(3);
            vel2=y(4);
            
            % coppie di feedforward
            tau_ffw=obj.model.inverseDynamics([sp_pos1;sp_pos2],[sp_vel1;sp_vel2],[sp_acc1;sp_acc2]);
%             tau_ffw=[0;0];
            
            % azioni di controllo
            u(1,1)=obj.ctrl_joint1.computeControlAction([sp_pos1 sp_vel1],[pos1 vel1],tau_ffw(1)); % giunto 1
            u(2,1)=obj.ctrl_joint2.computeControlAction([sp_pos2 sp_vel2],[pos2 vel2],tau_ffw(2)); % giunto 2
        end

    end

end