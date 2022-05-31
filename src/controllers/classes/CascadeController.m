classdef CascadeController < BaseController 
    
    properties  (Access = protected)
        PID_pos % PID loop esterno
        PID_vel % PID loop interno
    end

    methods

        function obj=CascadeController(st,Cv,Cp)
            obj@BaseController(st);
            obj.PID_pos=Cp;
            obj.PID_vel=Cv;
        end
        
        function setUMax(obj,umax)
            % setto limiti di coppia controllori giunti
            setUMax@BaseController(obj,umax);
            obj.PID_vel.setUMax(obj.umax);
            % solo il controllore interno è direttamente
            % collegato all'attuatore, che è soggetto a saturazione
        end
        
        function initialize(obj)
            %inizializzo controllori
            obj.PID_pos.initialize;
            obj.PID_vel.initialize;
        end

        function u=computeControlAction(obj,reference,y,tau_ffw)
            % calcolo azione di controllo

            % misure
            pos=y(1);
            vel=y(2);
            
            % setpoint
            sp_pos=reference(1);
            sp_vel=reference(2);
            
            % azioni di controllo
            u_pos=obj.PID_pos.computeControlAction(sp_pos,pos,obj.PID_vel.getSat_flag,vel,sp_vel); % controllore 1
            u=obj.PID_vel.computeControlAction(u_pos,vel,[],[],tau_ffw); % controllore 2
        end

    end

end