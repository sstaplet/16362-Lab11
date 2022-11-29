classdef controller < handle
    % The controller calculates the feedback for a robot given it's
    % reference and actual (estimated) position

    properties
        trajectory;
        lambda;
        use;
        logOn;

        startPose;

        i;
        times;
        ref_traj;
        act_traj;
        err;

        turning;
    end

    methods
        function obj = controller(lambda, use, logOn)
            obj.lambda = lambda;
            obj.logOn = logOn;
            obj.use = use;
            obj.turning = false;

            if obj.logOn
                obj.i = 0;
                obj.times = zeros(1,1000);
                obj.ref_traj = zeros(3, 1000);
                obj.act_traj = zeros(3,1000);
                obj.err = zeros(3,1000);
            end
        end

        function initialize(obj, robotTrajectory, startPose)
            % Initialize controller to start motion
            obj.trajectory = robotTrajectory;
            obj.startPose = startPose;
        end

        function [u_v, u_w] = getFeedback(obj, time, robotPose)
            % Return the feedback for a given trajectory at a given time
            % with the specified robot pose
            % - time: trajectory time
            % - robotPose: the pose of the robot at the time given
            % - u_v: the velocity feedback 
            % - u_w: the omega feedback

            % Calculate the reference pose at time 
            ref_pose = obj.trajectory.getPoseAtTime(time)';
            
            refPose = Pose(ref_pose(1), ref_pose(2), ref_pose(3));

            transform2 = refPose.bToA();
            transform1 = obj.startPose.bToA();
            transform = transform1 * transform2;

            ref_pose = Pose.matToPoseVec(transform);

            % Get the error btwn reference and robot
            err_pose = ref_pose' - robotPose;

            err_theta = atan2(sin(err_pose(3)), cos(err_pose(3)));

            robotTheta = ref_pose(3);
            
            rot_matrix = [cos(robotTheta), -sin(robotTheta);
                          sin(robotTheta), cos(robotTheta)];

            body_error = rot_matrix^-1 * [err_pose(1); err_pose(2)];

            if obj.logOn
                obj.i = obj.i + 1;
                obj.times(obj.i) = time;
                obj.ref_traj(:,obj.i) = ref_pose;
                obj.act_traj(:,obj.i) = robotPose;
                obj.err(:,obj.i) = [body_error(1), body_error(2), err_theta];
            end

            kx = 1.0 / obj.lambda;
            ky = 2.0 / (obj.lambda * obj.lambda);
            ktheta = 2.0 / obj.lambda;

            if (obj.turning)
                kx = 0.0;
                ky = 0.0;
            end

            % Calculate the feedback
            u_v = body_error(1) * kx;
            u_w = body_error(2) * ky + ktheta * err_theta;
        end
    end
end