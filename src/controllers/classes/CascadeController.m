classdef CascadeController < BaseController 
    
    properties  (Access = protected)
        PID_pos    % controllore PID loop esterno
        PID_vel    % controllore PID loop interno
    end

    methods

        function obj=CascadeController(st,Cv,Cp)
            obj@BaseController(st);
            obj.PID_pos=Cp;
            obj.PID_vel=Cv;
        end
        
        function setUMax(obj,umax)
            %% settaggio limiti di coppia controllori giunti
            setUMax@BaseController(obj,umax);
            obj.PID_vel.setUMax(obj.umax);    % controllore interno direttamente collegato all'attuatore, soggetto a saturazione
        end
        
        function initialize(obj)
            %% inizializzazione controllori
            % setta a zero l'azione integrale di ogni controllore
            obj.PID_pos.initialize;
            obj.PID_vel.initialize;
        end

        function u=computeControlAction(obj,reference,y,tau_ffw)
            %% calcolo azione di controllo
            % misure di posizione e velocità
            pos=y(1);
            vel=y(2);
            
            % setpoint di posizione e velocità
            sp_pos=reference(1);
            sp_vel=reference(2);
            
            % azioni di controllo
            u_pos=obj.PID_pos.computeControlAction(sp_pos,pos,obj.PID_vel.getSat_flag,vel,sp_vel);    % controllore esterno di posizione
            u=obj.PID_vel.computeControlAction(u_pos,vel,[],[],tau_ffw);                              % controllore interno di velocità
        end

    end

end