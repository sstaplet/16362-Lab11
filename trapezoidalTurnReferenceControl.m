classdef trapezoidalTurnReferenceControl < handle
    properties
        amax;
        vmax;
        angleChange;
        sgn;
        buffer;
    end
    methods
        function obj = trapezoidalTurnReferenceControl(amax, vmax, angleChange, sgn)
            obj.amax = amax;
            obj.vmax = vmax;
            obj.angleChange = angleChange;
            obj.sgn = sgn;
            obj.buffer = 0.5;
        end

        function w = getwAtTime(obj, timeNow)
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

            w = obj.sgn * uref;
            w = atan2(sin(w), cos(w));
        end

        function pose = getPoseAtTime(obj, time)
            t_ramp = obj.vmax / obj.amax;
            tf = obj.getTF();

            w = 0.0;

            if 2.0 * t_ramp > tf
                if timeNow < 0.0
                    w = 0.0;
                elseif timeNow > tf
                    w = obj.angleChange;
                elseif timeNow < tf / 2.0
                    w = obj.amax * timeNow;
                else
                    t = (tf*0.5) - (tf - time);
                    w = (((tf / 2.0) * obj.vmax) / 2.0) + (obj.vmax * t + (0.5 * (-obj.amax) * t * t));
                end
            else
                if time < t_ramp && time > 0.0
                    w = 1.0/2.0 * obj.amax * time * time;
                elseif (tf - time) < t_ramp && time < tf
                    t = t_ramp - (tf - time);
                    w = ((t_ramp * obj.vmax) / 2.0) + (obj.vmax * (tf - (2.0 * t_ramp))) + (obj.vmax * t + 0.5 * t * t * (-obj.amax));
                elseif 0.0 < time && time < tf
                    w = ((t_ramp * obj.vmax) / 2.0) + (obj.vmax * (time - t_ramp));
                elseif time < 0
                    w = 0.0;
                else
                    w = obj.angleChange;
                end
            end

            w = atan2(sin(w), cos(w));
            w = w * obj.sgn;
            pose = [0.0, 0.0, w];
        end

        function V = getVAtTime(obj,timeNow)
            V = 0.0;
        end

        function tf = getTF(obj)
            tf = (obj.angleChange + ((obj.vmax * obj.vmax) / obj.amax)) / obj.vmax;
        end

        function duration = getTrajectoryDuration(obj)
            tf = obj.getTF();
            duration = obj.buffer + tf;
        end
    end
end