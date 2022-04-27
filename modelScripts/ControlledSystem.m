classdef ControlledSystem < handle
    properties  (Access = protected)
        model
        controller
        time
        st
        controlled_output_index
        goal_output=1;
        IAE_baseline=0.55;
        settling_time_baseline=0.8;
        CE_baseline=0.72;
        OV_baseline=2;
    end
    methods  (Access = public)
        function obj=ControlledSystem(model)
            obj.model=model;
            obj.controller=[];
            obj.time=0;
            obj.st=model.getSamplingPeriod;
        end

        function setController(obj,controller,controlled_output_index)
            if nargin<3
                obj.controlled_output_index=1:obj.model.getOutputNumber;
            else
                obj.controlled_output_index=controlled_output_index;
            end
            obj.controller=controller;
            assert(obj.controller.getSamplingPeriod==obj.model.getSamplingPeriod,'Controller sampling period is wrong');
            obj.controller.setUMax(obj.model.getUMax)

        end

        function initialize(obj)
            rng shuffle
            obj.model.initialize;
            if ~isempty(obj.controller)
                obj.controller.inizialize;
            end
            obj.time=0;
        end

        function [y,t]=openloop(obj,control_action)
            obj.model.setScenario(1);
            t=obj.time;
            obj.time=obj.time+obj.st;
            y=obj.model.computeOutput;
            obj.model.updateState(control_action,t);
        end

        function [y,u,t]=step(obj,reference,u_feedforward)
            if (nargin<3)
                u_feedforward=zeros(obj.model.getInputNumber,1);
            end
            t=obj.time;
            obj.time=obj.time+obj.st;
            y=obj.model.computeOutput;
            assert(~isempty(obj.controller),'Controller is not set');
            u=obj.controller.computeControlAction(reference,y)+u_feedforward;
            obj.model.updateState(u,t);
        end

        function st=getSamplingPeriod(obj)
            st=obj.st;
        end

        function [score,results]=evalution(obj)

            for is=1:5
                results(is)=simulation(obj,is+1); %#ok<AGROW> 
                scores(is)=obj.computeScore(results(is)); %#ok<AGROW> 
            end
            score=mean(scores);
        end

        function [result]=simulation(obj,scenario)
            obj.initialize
            obj.model.setScenario(scenario);

            [t,reference]=generateTask(obj,scenario);
            y=zeros(length(t),obj.model.getOutputNumber);
            u=zeros(length(t),obj.model.getInputNumber);
            for idx=1:length(t)
                [y(idx,:),u(idx,:),t(idx,1)]=obj.step(reference(idx,:));
            end

            result.t=t;
            result.y=y;
            result.u=u;
            result.reference=reference;
        end

        function [t,reference]=generateTask(obj,scenario)
            t=(0:obj.st:20)';
            reference=zeros(length(t),obj.model.getOutputNumber);
        end
    end

    methods  (Access = protected)
        function score=computeScore(obj,result)
            score=0;
        end
    end

end
