classdef Pose < handle
    %pose A layer over a column vector that provides
    % access methods and associated homogeneous
    % transforms. For the purpose of naming the
    % homogeneous transforms, the pose is considered
    % to be that of frame b relative to frame a.
    properties(Constant)
        someConst = 24.56;
    end

    properties(Access = private)
        poseVec;
    end

    methods(Static = true)
        function error = poseError(ob1ToWld,ob2ToWld,len)
            %poseError Provides a scalar measure of pose error.
            %
            % Provides a scalar measure of all 3 axes of pose error
            % between two provided object poses. len is used to scale
            % orientation error.
            %
            % Compute pose of ob2 in ob1 frame
            ob2Wrtob1 = pose(ob1ToWld^-1*ob2ToWld);
            
            % compute error in all coordinates
            eX = ob2Wrtob1.x;
            eY = ob2Wrtob1.y;
            eT = ob2Wrtob1.th;
            if(eT > pi()/2)
                eT = eT - pi;
            elseif (eT < - pi()/2)
                eT = eT + pi;
            end
            %fprintf("pose_error: x:%f y:%f t:%f\n",eX,eY,eT);
            eT = eT*len;
            
            error = norm([eX eY eT]);
        end

        function vec = matToPoseVec(mat)
            % Convert a homogeneous transform into a
            % vector that can be passed to the
            % constructor for this class.
            x = mat(1,3);
            y = mat(2,3);
            w = atan2(-mat(1,2),mat(1,1));
            vec = [x ; y ; w];
        end
    end

    methods(Access = public)
        function obj = Pose(x,y,th)
            if(nargin == 3)
                obj.poseVec = [x ; y ; th];
            else
                obj.poseVec = [0 ; 0 ; 0];
            end
        end

        function x = x(obj); x = obj.poseVec(1); end

        function y = y(obj); y = obj.poseVec(2); end

        function th = th(obj); th = obj.poseVec(3); end

        function p = getPose(obj); p = obj.poseVec; end

        function mat = bToA(obj)
            %Returns the homogeneous transform that converts
            % coordinates from the b frame to the a frame.
            mat = zeros(3,3);
            x = obj.poseVec(1); y = obj.poseVec(2);
            th = obj.poseVec(3);
            mat(1,1) = cos(th); mat(1,2) = -sin(th);
            mat(1,3) = x; mat(2,1) = sin(th);
            mat(2,2) = cos(th); mat(2,3) = y;
            mat(3,1) = 0.0; mat(3,2) = 0.0; mat(3,3) = 1;
        end

        function mat = aToB(obj)
            %Returns the homogeneous transform that converts
            % coordinates from the a frame to the b frame.
            bTa = bToA(obj);
            mat = bTa^-1;
        end

        function mat = bToARot(obj)
            % Returns the rotation matrix that converts
            % coordinates from the b frame to the a frame.
            mat = zeros(2,2);
            th = obj.poseVec(3);
            mat(1,1) = cos(th); mat(1,2) = -sin(th);
            mat(2,1) = sin(th); mat(2,2) = cos(th);
        end

        function mat = aToBRot(obj)
            % Returns the rotation matrix that converts
            % coordinates from the a frame to the b frame.
            bTa = bToARot(obj);
            mat = bTa^-1;
        end
    end
end
