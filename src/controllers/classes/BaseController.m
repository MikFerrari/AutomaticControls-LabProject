classdef BaseController < handle

    properties  (Access = protected)
        st      % tempo di campionamento
        umax    % valore massimo variabile di controllo
    end

    methods

        function obj=BaseController(st)
            obj.st=st;
        end

        function inizialize(obj)
        end

        function setUMax(obj,umax)
            %% settaggio limite massimo azione di controllo
            obj.umax=umax;
        end

        function u=computeControlAction(obj,reference,y)
            %% calcolo azione di controllo
            u=0*obj.umax;
        end

        function st=getSamplingPeriod(obj)
            %% restituisce il tempo di campionamento del controllore discretizzato
            st=obj.st;
        end

    end

end