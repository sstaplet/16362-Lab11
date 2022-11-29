classdef TrajectoryFollower < handle
    % Will make the robot follow the specified trajectories from a given
    % start position
    % - loadTrajectory(trajectory, startPose)
    %       Call this before followTrajectory() to specify the trajectory
    %       for the robot to follow and the position the robot's supposed
    %       to be starting at (should be close to the robot's position
    %       before executing the trajectory).
    %
    % - followTrajectory()
    %       Call this to make the robot go along the trajectory specified
    %       by the last call to loadTrajectory()

    properties
        controller;
        trajectory;
        startPose;
        rIF;
        bufferTime = 0.2;
        W = robotModel.W;
        estRobot;
        logOn;

        ph1;
        ph2;
        ph3;
    end
    methods
        function obj = TrajectoryFollower(rIF, controller, estRobot, logOn)
            obj.rIF = rIF;
            obj.controller = controller;
            obj.estRobot = estRobot;
            obj.logOn = logOn;
        end

        function setLogInfo(obj, ph1, ph2, ph3)
            obj.ph1 = ph1;
            obj.ph2 = ph2;
            obj.ph3 = ph3;
        end

        function loadTrajectory(obj, trajectory, refStartPose)
            % Set the trajectory to follow and the start pose
            obj.trajectory = trajectory;
            obj.startPose = refStartPose;
            obj.controller.initialize(trajectory, refStartPose);
        end

        function followTrajectory(obj)
            % Follow the trajectory set in loadTrajectory
            firstItr = true;
            prevTime = 0.0;
            currTime = 0.0;
            startTime = 0.0;

            while currTime < obj.trajectory.getTrajectoryDuration() + obj.bufferTime
                pause(0.001);

                if firstItr
                    startTime = obj.rIF.toc();
                    firstItr = false;
                end

                currTime = obj.rIF.toc() - startTime;
                robotX = obj.estRobot.robotX;
                robotY = obj.estRobot.robotY;
                robotTheta = obj.estRobot.robotTheta;

                robotPose = [robotX, robotY, robotTheta];
                
                % Compute feedforward stuff
                v = obj.trajectory.getVAtTime(currTime);
                w = obj.trajectory.getwAtTime(currTime);
                
                % Compute feedback stuff
                u_v = 0.0;
                u_w = 0.0;
                if obj.controller.use
                    [u_v, u_w] = obj.controller.getFeedback(currTime, robotPose);
                end

                v = v + u_v;
                w = w + u_w;
                
                if isnan(v)
                    v = 0; w = 0;
                end
                
                vl = v - (obj.W / 2.0) * (w);
                vr = v + (obj.W / 2.0) * (w);

                [vl, vr] = robotModel.limitWheelVelocities([vl, vr], 0.98);
                obj.rIF.sendVelocity(vl, vr);

                if obj.logOn
                % Draw it
                    worldLineArray = worldModel.createXYAxes();
                    lidarSkip = 10; % Up to 30 should work
                    bodyPts = robotModel.bodyGraph();
                    ranges = obj.rIF.laser.LatestMessage.Ranges();
                    rImage = pointCloud(ranges,lidarSkip,true);
                    len = size(rImage.xArray,2);
                    points = [rImage.xArray ; rImage.yArray ; ones(1,len)];

                    latestPoseVec = [robotX; robotY; robotTheta];

                    worldBodyPts = pose(latestPoseVec)*bodyPts;
                    worldLidarPts = pose(latestPoseVec)*points;
                    set(obj.ph1, 'XData', worldLineArray(1,:), 'YData',worldLineArray(2,:));
                    set(obj.ph2, 'XData',worldBodyPts(1,:), 'YData',worldBodyPts(2,:));
                    set(obj.ph3, 'XData',worldLidarPts(1,:), 'YData',worldLidarPts(2,:));
                end

            end

            obj.rIF.stop();
            
        end
    end
end