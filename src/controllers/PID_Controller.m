classdef PID_Controller < BaseController 
    
    properties  (Access = protected)
        xi
        Kp
        Ki
        Kd
        Tf
        Kaw
        
    end

    methods

        function obj=PID_Controller(st,Kp,Ki,Kd,Tf,Kaw)
            obj@BaseController(st);
            obj.xi=0;
            obj.Kp=Kp;
            obj.Ki=Ki;
            obj.Kd=Kd;
            obj.Tf=Tf;
            obj.Kaw=Kaw; 
        end

        function initialize(obj)
            obj.xi=0;
        end

        function u=computeControlAction(obj,reference,y)
            e=reference-y;
            
            u=obj.xi+obj.Kp*e;
            if (u>obj.umax)
                usat=obj.umax;
            elseif (u<-obj.umax)
                usat=-obj.umax;
            else
                usat=u;
            end
            obj.xi=obj.xi+obj.Ki*obj.st*e+obj.Kaw*obj.st*(usat-u);
            
            u=usat;
        end

    end

end