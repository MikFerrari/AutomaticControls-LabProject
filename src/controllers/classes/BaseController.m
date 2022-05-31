classdef BaseController < handle
    properties  (Access = protected)
        st
        umax
    end
    methods
        function obj=BaseController(st)
            obj.st=st;
        end
        function inizialize(obj)
        end
        function setUMax(obj,umax)
            obj.umax=umax;
        end
        function u=computeControlAction(obj,reference,y)
            u=0*obj.umax;
        end
        function st=getSamplingPeriod(obj)
            st=obj.st;
        end
    end
end