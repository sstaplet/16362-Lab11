classdef odometryStateEstimator < handle
    % Gives estimates of the robot's speed, current position in real time
    % using the encoders - directly access the properties to get the robot
    % estimates

    properties
        initialLeftDist {mustBeNumeric}
        initialRightDist {mustBeNumeric}

        lastLeftDist {mustBeNumeric}
        lastRightDist {mustBeNumeric}
        encoderDist {mustBeNumeric}

        encoderStartTimeStamp {mustBeNumeric}
        encoderLastTimeStamp {mustBeNumeric}

        lastLeftVelocityEstimate {mustBeNumeric}
        lastRightVelocityEstimate {mustBeNumeric}

        robotX {mustBeNumeric}
        robotY {mustBeNumeric}
        robotTheta {mustBeNumeric}
        initialPose;

        W = robotModel.W;

        encoderDataStarted;
    end
    methods
        function obj = odometryStateEstimator(rIF, vecInit)
            rIF.encoders.NewMessageFcn=[];
            rIF.encoders.NewMessageFcn=@obj.encoderEventListener;
            
            obj.initialLeftDist = 0.0;
            obj.initialRightDist = 0.0;

            obj.lastLeftDist = 0.0;
            obj.lastRightDist = 0.0;
            obj.encoderDist = 0.0;

            obj.encoderStartTimeStamp = 0.0;
            obj.encoderLastTimeStamp = 0.0;

            obj.lastLeftVelocityEstimate = 0.0;
            obj.lastRightVelocityEstimate = 0.0;

            obj.robotX = vecInit(1);
            obj.robotY = vecInit(2);
            obj.robotTheta = vecInit(3);
            obj.initialPose = vecInit;
        end
        function obj = encoderEventListener(obj, handle, event)
            if(~isscalar(obj.encoderDataStarted))
                disp("Encoder data started");
                obj.encoderDataStarted = true;
                obj.encoderStartTimeStamp = raspbot.timeFromStamp(event.Header.Stamp);
            
                obj.initialLeftDist = event.Vector.X;
                obj.initialRightDist = event.Vector.Y;

                obj.lastLeftDist = 0.0;
                obj.lastRightDist = 0.0;
                obj.encoderDist = 0.0;

                obj.robotX = obj.initialPose(1);
                obj.robotY = obj.initialPose(2);
                obj.robotTheta = obj.initialPose(3);

                obj.encoderLastTimeStamp = 0.0;
            end
            
            leftEncoderDistance = event.Vector.X - obj.initialLeftDist; %(left wheel) 
            rightEncoderDistance = event.Vector.Y - obj.initialRightDist; %(right wheel)
            encoderDataTimeStamp = raspbot.timeFromStamp(event.Header.Stamp) - obj.encoderStartTimeStamp;
        
            dist = (leftEncoderDistance + rightEncoderDistance) / 2.0;
            prev_dist = (obj.lastLeftDist + obj.lastRightDist) / 2.0;
            ds = dist - prev_dist;

            dt = encoderDataTimeStamp - obj.encoderLastTimeStamp;
            dsl = leftEncoderDistance - obj.lastLeftDist;
            dsr = rightEncoderDistance - obj.lastRightDist;
        
            vl = dsl / dt;
            vr = dsr / dt;

            dssl = leftEncoderDistance - obj.lastLeftDist;
            dssr = rightEncoderDistance - obj.lastRightDist;

            obj.robotTheta = obj.robotTheta + (((dssr - dssl) / obj.W) / 2.0);
            obj.robotX = obj.robotX + cos(obj.robotTheta) * ds;
            obj.robotY = obj.robotY + sin(obj.robotTheta) * ds;
            obj.robotTheta = obj.robotTheta + (((dssr - dssl) / obj.W) / 2.0);
        
            obj.lastLeftDist = leftEncoderDistance;
            obj.lastRightDist = rightEncoderDistance;
            obj.encoderLastTimeStamp = encoderDataTimeStamp;
            obj.lastLeftVelocityEstimate = vl;
            obj.lastRightVelocityEstimate = vr;
            obj.encoderDist = (leftEncoderDistance + rightEncoderDistance) / 2.0;
        end
        
        function vec = getVec(obj)
            vec = [obj.robotX; obj.robotY; obj.robotTheta];
        end

        function correctPose(obj, vec)
            obj.robotX = vec(1);
            obj.robotY = vec(2);
            obj.robotTheta = vec(3);
        end
    end
end