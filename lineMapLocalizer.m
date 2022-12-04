classdef lineMapLocalizer < handle
    %mapLocalizer A class to match a range scan against a map in
    % order to find the true location of the range scan relative to
    % the map.
    
    properties(Constant)
        maxErr = 0.05; % 5 cm
        minPts = 5; % min # of points that must match
    end
    
    properties(Access = private)
    end
    
    properties(Access = public)
        lineArray;
        gain = 0.0;
        errThresh = 0.0;
        gradThresh = 0.0;
    end

    methods
        function obj = lineMapLocalizer(lineArray, gain, errThresh, gradThresh)
            %gain = 0.3;
            %errThresh = 0.01; % threshold on sqrt of fitError
            %gradThresh = 0.0005; % threshold on norm of gradient

            % create a lineMapLocalizer
            obj.lineArray = lineArray;
            obj.gain = gain;
            obj.errThresh = errThresh;
            obj.gradThresh = gradThresh;
        end

        function ro2 = closestSquaredDistanceToLines(obj,pi)
            % Find the squared shortest distance from pi to any line
            % segment in the supplied list of line segments.
            % pi is an array of 2d points
            % throw away homogenous flag
            pi = pi(1:2,:);
            r2Array = zeros((size(obj.lineArray,2)-1),size(pi,2));

            for i = 1:(size(obj.lineArray,2)-1)
                [r2Array(i,:) , ~] = closestPointOnLineSegment(pi,...
                obj.lineArray(1:2, i), obj.lineArray(1:2, i+1));
            end

            ro2 = min(r2Array,[],1);
        end

        function ids = throwOutliers(obj,pose,ptsInModelFrame)
            % Find ids of outliers in a scan.
            worldPts = pose.bToA()*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            ids = find(r2 > obj.maxErr*obj.maxErr);
        end

        function avgErr2 = fitError(obj,pose,ptsInModelFrame)
            % Find the variance of perpendicular distances of
            % all points to all lines
            % transform the points
            worldPts = pose.bToA()*ptsInModelFrame;
        
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            r2(r2 == Inf) = [];
            err2 = sum(r2);
            num = length(r2);

            if(num >= lineMapLocalizer.minPts)
                avgErr2 = err2/num;
            else
                % not enough points to make a guess
                avgErr2 = inf;
            end
        end

        function [err2_Plus0,J] = getJacobian(obj,poseVecIn,modelPts)
            % Computes the gradient of the error function
            err2_Plus0 = obj.fitError(poseVecIn, modelPts);
        
            eps = 1e-9; % THIS MAY BE TOO SMALL FOR YOU. TRY -4 IF SO

            dp = [eps ; 0.0 ; 0.0];
            poseVec = [poseVecIn.x(); poseVecIn.y(); poseVecIn.th()];
            newPoseVec = pose(poseVec + dp);
            err = obj.fitError(newPoseVec, modelPts);
            dEdx = (err - err2_Plus0) / eps;

            dp = [0.0 ; eps ; 0.0];
            newPoseVec = pose(poseVec + dp);
            err = obj.fitError(newPoseVec, modelPts);
            dEdy = (err - err2_Plus0) / eps;

            dp = [0.0 ; 0.0 ; eps];
            newPoseVec = pose(poseVec + dp);
            err = obj.fitError(newPoseVec, modelPts);
            dEdth = (err - err2_Plus0) / eps;
        
            % Fill me in…
            J = [dEdx, dEdy, dEdth];
        end

        function [success, outPose] = refinePose(obj, inPose, ptsInModelFrame, maxIters)
            % refine robot pose in world (inPose) based on lidar
            % registration. Terminates if maxIters iterations is
            % exceeded or if insufficient points match the lines.
            % Even if the minimum is not found, outPose will contain
            % any changes that reduced the fit error. Pose changes that
            % increase fit error are not included and termination
            % occurs thereafter.
            % Fill me in…

            ids = obj.throwOutliers(pose,ptsInModelFrame);
            modelPts = ptsInModelFrame(:,ids);

            outPose = inPose;
            k = obj.gain;
            success = false;

            for i=1:maxIters
                [err , J] = obj.getJacobian(outPose, modelPts);

                if err < obj.errThresh
                    success = true;
                    break;
                end

                outPose = outPose - k .* J;
            end
        end

    end
end
