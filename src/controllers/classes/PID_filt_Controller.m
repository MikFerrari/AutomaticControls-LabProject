classdef PID_filt_Controller < BaseController 
    
    properties  (Access = protected)
        xi                          % azione integrale
        Ki                          % coefficiente azione integrale
        n                           % ordine controllore
        u_PD_past                   % buffer azioni di controllo passate
        e_past                      % buffer errori passati
        A                           % coefficienti azione di controllo eq. differenze
        B                           % coefficienti errori eq. differenze
        Kaw=0;                      % costante antiwindup
        Kaw_ext=0;                  % costante antiwindup esterna
        sat_flag=false;             % true se il controllore sta saturando 
    end

    methods

        function obj=PID_filt_Controller(st,Ki,coeff_num,coeff_den,Kaw,Kaw_ext)
            obj@BaseController(st);
            obj.xi=0;
            obj.Ki=Ki;
            obj.n=length(coeff_den)-1;
            obj.u_PD_past = zeros(obj.n,1); 
            obj.e_past = zeros(obj.n,1);
            obj.A=-coeff_den(2:end); 
            obj.B=coeff_num;
            obj.Kaw=Kaw;
            obj.Kaw_ext=Kaw_ext;
        end

        function initialize(obj)
            obj.xi=0;
        end

        function sat_flag=getSat_flag(obj)
            sat_flag=obj.sat_flag;
        end

        function u=computeControlAction(obj,reference,y,TR_M,tracked_val,u_ffw)
            %% Implementazione equazione alle differenze (PD component)
            e_k = reference-y;               % errore corrente
            u_PD_k = obj.B(1)*e_k;           % azione di controllo istante presente
            
            for i = 1:obj.n
                u_PD_k = u_PD_k+obj.B(1+i)*obj.e_past(i)+obj.A(i)*obj.u_PD_past(i);
            end
            
            %% Aggiornamento buffer di errori e azioni di controllo passate
            for i = obj.n:-1:2    % ad ogni passo far scalare di 1 posto gli elementi di ciascun vettore dei valori passati
                obj.e_past(i) = obj.e_past(i-1);
                obj.u_PD_past(i) = obj.u_PD_past(i-1);
            end
            
            % inserire all'inizio dei buffer i due valori dell'iterazione corrente
            obj.e_past(1) = e_k;
            obj.u_PD_past(1) = u_PD_k;

            %% Calcolo azione di controllo totale (PD+integrale+ffw)
            u=obj.xi+u_PD_k+u_ffw;

            %% Controllo saturazione interna
            if (u>obj.umax)
                usat=obj.umax;
                obj.sat_flag=true;
            elseif (u<-obj.umax)
                usat=-obj.umax;
                obj.sat_flag=true;
            else
                usat=u;
                obj.sat_flag=false;
            end           

            %% Aggiornamento azione integrale e ANTIWINDUP 
            if TR_M
                obj.xi=obj.xi+obj.Ki*obj.st*e_k+obj.Kaw*obj.st*(usat-u)+obj.Kaw_ext*obj.st*(tracked_val-u); % saturazione interna ed esterna
            else
                obj.xi=obj.xi+obj.Ki*obj.st*e_k+obj.Kaw*obj.st*(usat-u); % solo saturazione interna
            end

            u=usat; % azione di controllo saturata effettivamente applicata
            
        end

    end

end