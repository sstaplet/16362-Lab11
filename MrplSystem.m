classdef MrplSystem < handle
    % The MRPL system executes trajectories
    % Call executeTrajectoryToRelativePose() to drive the robot somewhere

    properties
        estRobot;
        maxVelocity;
        rIF;
        currentRefPose;
        trajectoryFollower;
        ctrler;
        rstTraj;
        perception;
        rob;
        scr_fig;
    end
    
    methods(Static = true)
        function acqPoseVec = acquisitionPose(objPoseVec, ...
            robFrontOffset,objFaceOffset,moreOffset)
            % Determine where the robot needs to be in order to pick up an
            % object. Accounts for the (few mm) width of the target from
            % detected surface to front face.
            % 
            % objPoseVec is the pose of the object (sail face or model
            % frame) relative to the sensor
            %
            % robFrontOffset is the offset from the robot frame to the
            % robot front surface
            %
            % objFaceOffset is the distance from the provided object pose
            % to the object front surface. This is either:
            % a) the sail-to-front distance when using perception to see
            % the object or
            % b) the (slightly longer by half a sail thickness) model-to-
            % front distance used when the real model frame pose is
            % provided from a map.
            %
            % moreOffset is the additional offset back from the
            % object face frame at which to place the front surface of the
            % robot. A notional value of that is slightly more than the
            % length of the forks, but it could be much more than that
            % if you wish to look at the object again and plan a new
            % trajectory.
            x = objPoseVec(1); y = objPoseVec(2); th = objPoseVec(3);
            offset = robFrontOffset + objFaceOffset + moreOffset;
            tsr = [1 0 0; 0 1 0; 0 0 1];
            tos = [cos(-th) sin(-th) x; -sin(-th) cos(-th) y; 0 0 1];
            tgo = [1 0 -offset; 0 1 0; 0 0 1];
            tgr = tsr*tos*tgo;
            acqPoseVec = [tgr(1,3); tgr(2,3); th];
        end

        function dist = findDist(p1,p2)
            diff = p2-p1;
            dist = sqrt(diff(1)^2 + diff(2)^2);
        end
    end

    methods
        function obj = MrplSystem(rIF, logOn, rstTraj, vecInit, k, lineArray, gain, errThresh, gradThresh, useMap, rob)
            obj.estRobot = stateEst(rIF, vecInit, k, lineArray, gain, errThresh, gradThresh, useMap);
            obj.maxVelocity = 0.2;
            obj.rIF = rIF;
            obj.currentRefPose = Pose(vecInit(1), vecInit(2), vecInit(3));
            obj.rstTraj = rstTraj;
            obj.ctrler = controller(0.25, true, logOn);
            obj.trajectoryFollower = TrajectoryFollower(rIF, obj.ctrler, obj.estRobot, logOn);
            obj.perception = pointCloudPerceptor2();
            obj.rob = rIF;
            obj.scr_fig = rIF.scr_fig;
        end

        function addObjects(obj, shapes)
            obj.rIF.addObjects(shapes);
        end

        function [minPick, picks] = closestPose(obj, picks)
            vec = obj.estRobot.getVec();
            minDist = 10000;
            for i = 1 : size(picks, 2)
                dist = norm(picks(1:2,i)-vec(1:2,:));
                if dist < minDist
                    minPick = picks(:, i);
                    besti = i;
                    minDist = dist;
                end
            end
            picks = [picks(:,1:besti-1), picks(:, besti+1:size(picks,2))];
        end

        function executeTrajectory(obj, trajectory)
            obj.trajectoryFollower.loadTrajectory(trajectory, obj.currentRefPose);
            obj.trajectoryFollower.followTrajectory();
        end
        
        function moveRelDist(obj, dist, doControlPlotting)
            % move forward or backward a specified distance and stop.
            % make sure the velocity is such that the distance will take at
            % least a second
            vmax = min(0.2, abs(dist));
            amax = vmax * 3.0;
            
            sgn = 1.0;
            if dist < 0.0
                sgn = -1.0;
            end

            %create a trapezoidal trajectory and use robot trajectory
            traj = trapezoidalStepReferenceControl(amax, vmax, abs(dist), sgn);
            %use this trajectory to use moveRelDist
            obj.trajectoryFollower.loadTrajectory(traj, obj.currentRefPose);
            obj.trajectoryFollower.followTrajectory();
        end

        function turnRelAngle(obj, angle, doControlPlotting)
            % make sure the velocity is such that the distance will take at
            % least a second
            angle = atan2(sin(angle), cos(angle));
            vmax = min(4.0, abs(angle));
            amax = 3.0 * vmax;

            sgn = 1.0;
            if angle < 0
                sgn = -1.0;
            end

            traj = trapezoidalTurnReferenceControl(amax, vmax, abs(angle), sgn);
            
            obj.trajectoryFollower.loadTrajectory(traj, obj.currentRefPose);
            obj.trajectoryFollower.controller.turning = true;
            obj.trajectoryFollower.followTrajectory();

            obj.trajectoryFollower.controller.turning = true;
            % move a distance forward or backward
        end

        function executeTrajectoryToRelativePose(obj, xf, yf, thf, vmax, sgn)
            % Plan and execute a trajectory using odometry as feedback
            % The terminal pose is specified in robot-relative coordinates
            % based on where the robot is when this function is called.
            if obj.rstTraj
                obj.ctrler.i = 0;
            end
            
            %find straight line distance
            cp = [obj.currentRefPose.x(); obj.currentRefPose.y(); obj.currentRefPose.th()];
            dist = MrplSystem.findDist([xf;yf], [0;0]);

            %if the straight line distance is less than 7mm, this isn't
            %working ahhhhhh
            if (dist <= .0001 && thf == cp(3))
                obj.moveStraightLineDistWithTrapezoidalTrajectory(vmax,dist,sgn);
            else
                trajectory = cubicSpiralTrajectory.planTrajectory(xf, yf, thf, 1.0);
                trajectory.planVelocities(obj.maxVelocity);
                obj.executeTrajectory(trajectory);
            end
            th = obj.currentRefPose.th() + thf;
            obj.currentRefPose = Pose(obj.currentRefPose.x() + xf, obj.currentRefPose.y() + yf, atan2(sin(th), cos(th)));
        end

        function moveStraightLineDistWithTrapezoidalTrajectory(obj, vmax, dist, sgn)
            amax = 0.05 * 3;
            %create a trapezoidal trajectory and use robot trajectory
            traj = trapezoidalStepReferenceControl(amax, vmax, dist, 1.0);
            %use this trajectory to use moveRelDist
            obj.moveRelDist(traj);
        end

        function plotTrajectories(obj) 
            % Call this to plot the robot trajectory (blue) and the
            % reference trajectory (green) so 
            i = obj.ctrler.i;
            figure();
            hold on;
            plot(obj.ctrler.act_traj(1,1:i), obj.ctrler.act_traj(2,1:i), 'b');
            plot(obj.ctrler.ref_traj(1,1:i), obj.ctrler.ref_traj(2,1:i), 'g');
            hold off
        end

        function thisPose = getCurrentPose(obj)
            thisPose = obj.estRobot.getVec();
        end

        function [x, y, th] = findNearestSail(obj)
            r = obj.rIF.laser.LatestMessage.Ranges;
            [centroid, th] = obj.perception.findLineCandidate(r);
            x = centroid(1); y = centroid(2);
%             rfOffset = robotModel.frontOffset;
%             ofOffset = robotModel.objOffset;
%             moreOffset = robotModel.forks_length;
%             adjVec = MrplSystem.acquisitionPose([x; y; th], rfOffset,ofOffset, moreOffset);
%             x = adjVec(1); y = adjVec(2); th = adjVec(3);
        end

        function executeTrajectoryToAbsPose(obj,xfa,yfa,thfa,vmax,sgn)
            % Execute a trajectory using state estimator as feedback
            % Plan the trajectory. The terminal pose is specified
            % in world coordinates (not start relative).
            % fill me in

            goalPos = Pose(xfa, yfa, thfa);
            currPos = obj.getCurrentPose;
            currPose = Pose(currPos(1), currPos(2), currPos(3));

            transform2 = goalPos.bToA();
            transform1 = currPose.aToB();
            transform = transform1 * transform2;

            relPos = Pose.matToPoseVec(transform);
            %relPos = goalPos.getPose();

            trajectory = cubicSpiralTrajectory.planTrajectory(relPos(1), relPos(2), relPos(3), sgn);
            trajectory.planVelocities(vmax);
            obj.trajectoryFollower.loadTrajectory(trajectory, currPose);
            obj.trajectoryFollower.followTrajectory();

            obj.currentRefPose = goalPos;
        end

        function [acq_pose, nominal_pose_vec] = pickDropObject(obj, object_pose, isPick)
            vmax = obj.maxVelocity;
            vec = obj.estRobot.getVec();
            if isPick
                x = vec(1); y = vec(2); th = vec(3);
                % Generates a trajectory and drives the robot to the acquisition pose.
                poseVec = [object_pose.x(); object_pose.y(); object_pose.th()];
                acq_pose = MrplSystem.acquisitionPose(poseVec, robotModel.frontOffset,...
                    robotModel.objOffset, robotModel.forks_length + .175);
                % Turns the robot, in place, to face the origin of the frame at the acquisition pose.
                % Do this only if the angle is relatively large because (later step) trajectory
                % generation can do small amounts of turning for you.
                turn_angle = atan2(acq_pose(2) - y,acq_pose(1) - x) - th;
                obj.turnRelAngle(turn_angle, false);
                vec = obj.estRobot.getVec();
                th = vec(3);
                if th / poseVec(3) < 0
                    poseVec(3) = -poseVec(3);
                end
                acq_pose = MrplSystem.acquisitionPose(poseVec, robotModel.frontOffset,...
                    robotModel.objOffset, robotModel.forks_length + .175);
                obj.executeTrajectoryToAbsPose(acq_pose(1), acq_pose(2), ...
                    acq_pose(3), vmax, 1);

                % Finds the object in a laser image taken at the acquisition pose.
                [x, y, th] = obj.findNearestSail();
                endGoal = [x;y;th];

                goalPos = Pose(x, y, th);
                currPos = obj.getCurrentPose;
                currPose = Pose(currPos(1), currPos(2), currPos(3));
    
                transform2 = goalPos.bToA();
                transform1 = currPose.bToA();
                transform = transform1 * transform2;

                absoluteGoalPos = Pose.matToPoseVec(transform);

                % Computes an ideal pickup pose based on positioning the robot fork base plate 7
                % cm or so directly in front of the pallet face to make fork insertion a simple straight
                % motion. If this pose is more than about 4 cm from your present position, go there.
                nominal_pose_vec = MrplSystem.acquisitionPose(endGoal, robotModel.frontOffset,...
                    robotModel.objOffset, robotModel.forks_length + .007);
                obj.executeTrajectoryToRelativePose(nominal_pose_vec(1), ...
                    nominal_pose_vec(2), nominal_pose_vec(3), vmax, 1.0);

                obj.rIF.stop();
                pause(0.5);

%                 goalPos = Pose(absoluteGoalPos(1), absoluteGoalPos(2), absoluteGoalPos(3));
%                 currPos = obj.getCurrentPose;
%                 currPose = Pose(currPos(1), currPos(2), currPos(3));
%     
%                 transform2 = goalPos.bToA();
%                 transform1 = currPose.aToB();
%                 transform = transform1 * transform2;
    
                %relPos = Pose.matToPoseVec(transform);

                %obj.executeTrajectoryToRelativePose(.01, 0, 0, .01, 1);
                obj.moveRelDist(0.063, false)
            else
                vec = obj.estRobot.getVec();
                x = vec(1); y = vec(2); th = vec(3);
                % Generates a trajectory and drives the robot to the acquisition pose.
                poseVec = [object_pose.x(); object_pose.y(); object_pose.th()];
                acq_pose = MrplSystem.acquisitionPose(poseVec, robotModel.frontOffset,...
                    robotModel.objOffset, .007);
                % Turns the robot, in place, to face the origin of the frame at the acquisition pose.
                % Do this only if the angle is relatively large because (later step) trajectory
                % generation can do small amounts of turning for you.
                turn_angle = atan2(acq_pose(2) - y,acq_pose(1) - x) - th;
                obj.turnRelAngle(turn_angle, false);
                obj.executeTrajectoryToAbsPose(acq_pose(1), acq_pose(2), ...
                    acq_pose(3), vmax, 1);
            end
            if isPick
                obj.forkUp;
            else
                obj.forkDown;
            end
            pause(1);
        end

        function forkUp(obj)
            obj.rIF.forksUp();
        end
        function forkDown(obj)
            obj.rIF.forksDown();
        end

        function shutdown(obj)
            obj.rIF.stopLaser();
            obj.rIF.stop();
        end
    end
end