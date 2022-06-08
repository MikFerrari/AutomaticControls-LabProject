classdef RoboticSystem < MechanicalSystem
    properties  (Access = protected)
        A
        B
        C
        robot
        njoints
        hm
        cm
        elastic_joints
        JmInv
        k
        h
        forwardDynamics
    end

    methods  (Access = public)
        function obj=RoboticSystem(st,robot,hm,cm,umax,elastic_joints,Jm,k,h)
            obj@MechanicalSystem(st); % uso il costruttore parente
            robot.DataFormat = 'column';
            obj.robot=robot;

            obj.njoints=length(robot.randomConfiguration);
            obj.num_input=obj.njoints;
            obj.num_output=2*obj.njoints;

            obj.hm=diag(hm);
            obj.cm=cm;
            obj.umax=umax;
            if (nargin<6)
                elastic_joints=false;
            end
            obj.elastic_joints=elastic_joints;
            if elastic_joints
                obj.order=obj.njoints*4; % pos,vel del motore e del link
                obj.JmInv=diag(1./Jm);
                obj.k=diag(k);
                obj.h=diag(h);
            else
                obj.order=obj.njoints*2; % pos vel del link
            end

            obj.x0=zeros(obj.order,1);
            obj.x=zeros(obj.order,1);
            obj.forwardDynamics=@(q,qd,tau)forwardDynamics(obj.robot,ql,qld,tau); % usa la dinamica diretta non autogenerata (lenta)
        end

        function setForwardDynamics(obj,forwardDynamics)  % usa la din.diretta autogenerata
            obj.forwardDynamics=forwardDynamics;
        end

        function show(obj) % visualizzare il robot 
            ql=obj.x(1:obj.njoints);
            show(obj.robot,ql);
        end

        function output_names=getOutputName(obj)
            for idx=1:obj.njoints
                output_names{idx}=sprintf('position_%d',idx);
                output_names{idx+obj.njoints}=sprintf('velocity_%d',idx);
            end
        end

        function input_names=getInputName(obj)
            for idx=1:obj.njoints
                input_names{idx}=sprintf('torque_%d',idx);
            end
        end
    end



    methods  (Access = protected)
        function Dx=stateFunction(obj,x,u,t)

            ql=x(1:obj.njoints);  % pos.link   
            qld=x((1:obj.njoints)+obj.njoints); %vel.link
            if obj.elastic_joints
                qm=x((1:obj.njoints)+2*obj.njoints); % pos.motore
                qmd=x((1:obj.njoints)+3*obj.njoints); % vel.motore

                ulink=obj.k*(qm-ql)+obj.h*(qmd-qld);  % coppia elastica

                qldd = obj.forwardDynamics(ql,qld,ulink);  % acc. link = dinamica diretta
                qmdd = obj.JmInv*(u-ulink-obj.hm*qmd);  % Jm*acc.motore= sum(coppie)
                Dx=[qld;qldd;qmd;qmdd];
            else
                qldd = obj.forwardDynamics(ql,qld,u); %% din.diretta   acc=.......()
                Dx=[qld;qldd];
            end
        end
        function y=outputFunction(obj)
            if (obj.elastic_joints)
                y=obj.x(2*obj.njoints+(1:2*obj.njoints));
            else
                y=obj.x;
            end
        end
    end
end