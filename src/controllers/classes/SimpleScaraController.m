classdef SimpleScaraController < BaseController
    
    properties  (Access = protected)
        % tuning empirico e grossolano: ci basta la stabilità del sistema
        % attorno al set-point, senza mirare a buone prestazioni
        Kp=zeros(2);
        Kv=zeros(2);
    end

    methods

        function obj=SimpleScaraController(st,Kp_diag,Kv_diag)
            obj@BaseController(st);
            if length(Kp_diag) ~= 2 || length(Kv_diag) ~= 2
                error('Wrong number of Kp and/or Kv elements! Specify 2 elements for each one.')
            end
            obj.Kp=diag(Kp_diag);
            obj.Kv=diag(Kv_diag);
        end

        function u=computeControlAction(obj,reference,y)

            % PD che controlla posizione del motore usando la coppia.
            % Semplice e non performante. Non adatto al controllo, ma
            % comodo per l'identificazione

            % u = Kp*errore_posizione + Kv*errore_velocità

            reference_position=reference(1:2);
            reference_velocity=reference(3:4);
            position=y(1:2);
            velocity=y(3:4);
            u=obj.Kp*(reference_position-position)+...
                obj.Kv*(reference_velocity-velocity);
            u=min(max(u,-obj.umax),obj.umax); % saturazione azione di controllo se minore del minimo e maggiore del massimo

            % non necessario l'anti-windup poichè non c'è alcun integratore
        end

    end

end