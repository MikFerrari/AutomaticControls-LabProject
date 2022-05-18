classdef SimpleController < BaseController 
    properties  (Access = protected)
        PIctrl1 % PI giunto 1
        PIctrl2 % PI giunto 2
    end
    methods
        function obj=SimpleController(st,Kp1,Ki1,Kp2,Ki2)
            obj@BaseController(st);
            obj.PIctrl1=PIController(st,Kp1,Ki1);
            obj.PIctrl2=PIController(st,Kp2,Ki2);
        
        end
        
        function setUMax(obj,umax)
            % setto limiti di coppia controllori giunti
            setUMax@BaseController(obj,umax);
            obj.PIctrl1.setUMax(obj.umax(1));
            obj.PIctrl2.setUMax(obj.umax(2));
        end
        
        function initialize(obj)
            %inizializzo controllori
            obj.PIctrl1.initialize;
            obj.PIctrl2.initialize;
        end

        function u=computeControlAction(obj,reference,y)
            % calcolo azione di controllo dei giunti

            % misure
            pos_jnt1=y(1);
            pos_jnt2=y(2);
            vel_jnt1=y(3);
            vel_jnt2=y(4);
            
            % setpoint
            sp_pos_jnt1=reference(1);
            sp_pos_jnt2=reference(2);
            
            sp_vel_jnt1=reference(3);
            sp_vel_jnt2=reference(4);
            
            sp_acc_jnt1=reference(5);
            sp_acc_jnt2=reference(6);
            
            % controllore 1
            u(1,1)=obj.PIctrl1.computeControlAction(sp_pos_jnt1,pos_jnt1);
            % controllore 2
            u(2,1)=obj.PIctrl2.computeControlAction(sp_pos_jnt2,pos_jnt2);
        end
    end
end