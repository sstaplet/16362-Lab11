classdef stateEstimator < odometryStateEstimator
    properties
        k;
        localizer;
        useMap;
    end

    methods
        function obj = stateEstimator(rIF, vecInit, k, ...
                lines_p1, lines_p2, gain, errThresh, gradThresh, useMap)   
            obj = obj@odometryStateEstimator(rIF,vecInit);
            obj.useMap = useMap;
            obj.k = k;
            obj.localizer = lineMapLocalizer(lines_p1, lines_p2, gain, errThresh, gradThresh);
        end

        function vec = getVec(obj)
            vec = [obj.robotX; obj.robotY; obj.robotTheta];
        end
        
        function vec = processRangeImage(obj, points, maxIters)
            if obj.useMap
                vec = pose(obj.getVec);
                [success, lidVec] = obj.localizer.refinePose(vec, points, maxIters);
                if success
                    pErr = lidVec - vec;
                    pErr(3) = atan2(sin(pErr(3)),cos(pErr(3)));
                    vec = vec + pose(obj.k*[pErr.x(); pErr.y(); pErr.th()]);
                    vec(3) = atan2(sin(vec(3)),cos(vec(3)));
                    obj.correctPose(vec);
                end
            end
        end
    end
end