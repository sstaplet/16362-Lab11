classdef trapezoidalStepReferenceControl < handle
    properties
        amax;
        vmax;
        dist;
        sgn;
        buffer;
    end
    methods
        function obj = trapezoidalStepReferenceControl(amax, vmax, dist, sgn)
            obj.amax = amax;
            obj.vmax = vmax;
            obj.dist = dist;
            obj.sgn = sgn;
            obj.buffer = 0.5;
        end

        function w = getwAtTime(obj, time)
            w = 0.0;
        end

        function pose = getPoseAtTime(obj, time)
            t_ramp = obj.vmax / obj.amax;
            tf = obj.getTF();

            if 2.0 * t_ramp > tf
                if timeNow < 0.0
                    x = 0.0;
                elseif timeNow > tf
                    x = obj.dist;
                elseif timeNow < tf / 2.0
                    w = obj.amax * timeNow;
                else
                    t = (tf*0.5) - (tf - time);
                    x = (((tf / 2.0) * obj.vmax) / 2.0) + (obj.vmax * t + (0.5 * (-obj.amax) * t * t));
                end
            else
                if time < t_ramp && time > 0.0
                    x = 1.0/2.0 * obj.amax * time * time;
                elseif (tf - time) < t_ramp && time < tf
                    t = t_ramp - (tf - time);
                    x = ((t_ramp * obj.vmax) / 2.0) + (obj.vmax * (tf - (2.0 * t_ramp))) + (obj.vmax * t + 0.5 * t * t * (-obj.amax));
                elseif 0.0 < time && time < tf
                    x = ((t_ramp * obj.vmax) / 2.0) + (obj.vmax * (time - t_ramp));
                elseif time < 0
                    x = 0.0;
                else
                    x = obj.dist;
                end
            end

            x = x * obj.sgn;
            pose = [x, 0.0, 0.0];

        end

        function V = getVAtTime(obj,timeNow)
            tf = obj.getTF();

            t_ramp = obj.vmax / obj.amax;

            if 2.0 * t_ramp > tf
                if timeNow < 0.0 || timeNow > tf
                    uref = 0.0;
                elseif timeNow < tf / 2.0
                    uref = obj.amax * timeNow;
                else
                    uref = obj.amax * (tf - timeNow);
                end

            else
                if timeNow < t_ramp && timeNow > 0.0
                uref = obj.amax * timeNow;
                elseif (tf - timeNow) < t_ramp && timeNow < tf
                     uref = obj.amax * (tf - timeNow);
                elseif 0.0 < timeNow && timeNow < tf
                    uref = obj.vmax;
                else
                    uref = 0.0;
                end
            end
            
            V = obj.sgn * uref;
        end

        function tf = getTF(obj)
            tf = (obj.dist + ((obj.vmax * obj.vmax) / obj.amax)) / obj.vmax;
        end

        function duration = getTrajectoryDuration(obj)
            tf = obj.getTF();
            duration = obj.buffer + tf;
        end
    end
end