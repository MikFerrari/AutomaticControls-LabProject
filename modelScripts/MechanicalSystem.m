classdef MechanicalSystem < handle
    properties  (Access = protected)
        x; % state vector
        x0; % initial value of the state vector
        st; % sampling period
        sigma_y=0; % noise standard deviation (if MIMO, it is a vector)
        t=0; % time
        umax=5; % maximum input

        order=2; % system order
        num_output=1; % number of system outputs
        num_input=1; % number of system inputs
        scenario=1;


    end

    methods  (Access = public)

        % obj=MechanicalSystem(st)
        % create a mechanical model with sampling period st
        function obj=MechanicalSystem(st)
            obj.st=st;
            obj.x=zeros(2,1);
            obj.x0=zeros(2,1);
            obj.order=2;
            obj.num_input=1;
            obj.num_output=1;
        end

        % (re)initialize the system to initial conditions
        function initialize(obj)
            obj.x=obj.x0;
            obj.t=0;
        end

        function setScenario(obj,scenario)
            obj.scenario=scenario;
        end

        % compute y=g(x,u)
        function y=computeOutput(obj)
            y=obj.outputFunction();
        end

        % solve derivative(x)=f(x,u) during sampling period
        function updateState(obj,u,t)
            usat=obj.saturationControlAction(u);
            obj.odeSolver(usat,obj.st,t);
        end


        % return system sampling period
        function st=getSamplingPeriod(obj)
            st=obj.st;
        end

        % return number of system outputs
        function num_output=getOutputNumber(obj)
            num_output=obj.num_output;
        end

        % return number of system inputs
        function num_input=getInputNumber(obj)
            num_input=obj.num_input;
        end

        function umax=getUMax(obj)
            umax=obj.umax;
        end

        function output_names=getOutputName(obj)
            for idx=1:obj.num_output
                output_names{idx}=sprintf('output_%d',idx);
            end
        end

        function input_names=getInputName(obj)
            for idx=1:obj.num_input
                input_names{idx}=sprintf('input_%d',idx);
            end
        end

    end

    methods  (Access = protected)

        % derivative(x)=f(x,u)
        function Dx=stateFunction(obj,x,u,t)
            Dx=0;
        end

        % ouput=g(x,u)
        function y=outputFunction(obj)
            y=obj.x+obj.sigma_y.*randn(length(obj.sigma_y),1);
        end

        % ODE solver (using RK4 with integration step = 0.1 st)
        function odeSolver(obj,u,st,t)
            n=1;
            dt=st/n;
            % Runge Kutta 4
            for idx=1:n
                k_1 = obj.stateFunction(obj.x,u,t);
                k_2 = obj.stateFunction(obj.x+0.5*dt*k_1,u,t);
                k_3 = obj.stateFunction(obj.x+0.5*dt*k_2,u,t);
                k_4 = obj.stateFunction(obj.x+k_3*dt,u,t);
                obj.x=obj.x+(1/6)*(k_1+2*k_2+2*k_3+k_4)*dt;
            end

        end

        % saturate control action
        function usat=saturationControlAction(obj,u)
            for iu=1:obj.num_input
                if (u(iu)>obj.umax(iu))
                    usat(iu,1)=obj.umax(iu);
                elseif (u(iu)<-obj.umax(iu))
                    usat(iu,1)=-obj.umax(iu);
                else
                    usat(iu,1)=u(iu);
                end
            end
        end
    end
end
