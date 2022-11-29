classdef pointCloudPerceptor2 < rangeImage
   properties
       maxObjectPosnError;
       minPointsRequiredSail;
       sailWidth;
       sailWidthLenErr;
       sailWidthLb;
       sailWidthUb;
       xArray;
       yArray;
   end
   methods(Static)
       function [ x, y, th] = irToXy( i, r )
           % irToXy finds position and bearing of a range pixel endpoint
           % Finds the position and bearing of the endpoint of a range pixel in the plane.
           % Fill in code here
           th = (i - 6) .* (pi ./ 180.0);
           xx = cos(th);
           x = r.* cos(th);
           y = r.* sin(th);
       end

   end
   methods
       function [m, b] = getBestFitLine(obj, x, y)
           avgX = mean(x);
           avgY = mean(y);
           xsMinusAvg = x - avgX;
           ysMinusAvg = y - avgY;
           m = (xsMinusAvg' * ysMinusAvg) / (xsMinusAvg' * xsMinusAvg);
           b = avgY - (m * avgX);
       end

       function obj = pointCloudPerceptor2(varargin)
           % Constructs a range image.
           %
           %   Arguments "varargin" are compatible with range sensor and,
           %   if present, specify the pose of the sensor on the robot.
           %
           obj@rangeImage(varargin{:}); 
           obj.sailWidth = 0.127;
           obj.maxObjectPosnError = 0.0635;
           obj.minPointsRequiredSail = 10;
           % might need adjustment with time
           obj.sailWidthLenErr = 0.016;
           obj.sailWidthLb = obj.sailWidth - obj.sailWidthLenErr;
           obj.sailWidthUb = obj.sailWidth + obj.sailWidthLenErr;
       end

       function goodOnes = findSailLinePoints(obj, objPoseVec, desR)
           % Subtract element by element to produce coordinates
           % relative to central pixel (objPoseVec)
           xc = obj.xArray - objPoseVec(1);
           yc = obj.yArray - objPoseVec(2);
           % compute radius of each point from the object center pixel
           rangeImgSqr = xc.*xc + yc.*yc;
           % clear those points in each column that exceed
           % desired radius
           goodOnes = rangeImgSqr < desR^2;
%             if(isempty(obj.xArray))
%                 fprintf('pointCloudPerceptor: All data is BAD !!\n');
%             end
       end

       function projectedPoint = projection(obj, x, y, m, b)
           perpSlope = -1 / m;
           yInt = -perpSlope * x + y;

           xIntersection = (yInt - b) / (m - perpSlope); 
           yIntersection = perpSlope * xIntersection + yInt; 
           projectedPoint = [xIntersection, yIntersection];
       end
       
       function [x, y, cx, cy, lengthEst] = findLengthandCenter(obj, x,y)
           %estimate the length of the points
           [m, b] = obj.getBestFitLine(y, x);

           randomY = mean(y);
           randomX = m * randomY + b;

           distSq = (x - randomX).^2 + (y - randomY).^2;
           maxDist = max(distSq);
           maxDistInd = find(distSq==maxDist);

           end1 = obj.projection(y(maxDistInd), x(maxDistInd), m, b);

           distSq = (x - end1(2)).^2 + (y - end1(1)).^2;
           maxDist = max(distSq);
           maxDistInd = find(distSq==maxDist);

           end2 = obj.projection(y(maxDistInd), x(maxDistInd), m, b);

           lengthEst = sqrt(sum(((end2 - end1).^2)));
           
           %find the centroid based on the ends of the line
           center = (end1 + end2) / 2.0;
           cx = center(2);
           cy = center(1);
            
           %adjust x and y by centroid
           x = x - cx;
           y = y - cy;
       end

       function [lambda, Ixx, Iyy, Ixy] = momentRoutine(obj, x, y, numPts)
           Ixx = x' * x;
           Iyy = y' * y;
           Ixy = - x' * y;
           Inertia = [Ixx Ixy;Ixy Iyy] / numPts; % normalized
           lambda = eig(Inertia);

           % Calculate lambda
           lambda = sqrt(lambda)*1000.0;
       end

       function [centroid, th] = findLineCandidate(obj, ranges)
           % go through the ranges
           is = 1:length(ranges);
           rs = ranges';
           % find the x,y,th distance from sensor
           [xs, ys, ts] = pointCloudPerceptor2.irToXy(is, rs);

           % grab the okay points data
           okPoints = rs > 0.06 & rs < 4.0;
           obj.iArray = is(okPoints);
           obj.rArray = rs(okPoints);
           obj.xArray = xs(okPoints);
           obj.yArray = ys(okPoints);
           obj.tArray = ts(okPoints);

            %set an arbitrary centroid and th
           centroid = [-1, -1];
           th = -500;

           % go through all the okay points
           for i=1:length(obj.rArray)
               %grab the points within saildistance from this one
               xx = obj.xArray(i);
               yy = obj.yArray(i);
               goodOnes = obj.findSailLinePoints([xx, yy], obj.maxObjectPosnError);

               %grab the data from the good points
               is = obj.iArray(goodOnes); % this is "logical indexing"
               ts = obj.tArray(goodOnes);
               rs = obj.rArray(goodOnes);
               x = obj.xArray(goodOnes)';
               y = obj.yArray(goodOnes)';

               %immediately disqualify as centroid if not enough pts
               numPts = length(x);
               if numPts < obj.minPointsRequiredSail
                   % There's not enough points for it to be considered a
                   % valid sail centroid
                   continue
               end
               
               %estimate the length of the points
               [x, y, cx, cy, lengthEst] = obj.findLengthandCenter(x,y);
              
               % Calculate lambda
               [lambda, Ixx, Iyy, Ixy] = obj.momentRoutine(x, y, numPts);
               
               if lambda(1) < 1.3
                   % length checkkk
                   if lengthEst > obj.sailWidthLb && lengthEst < obj.sailWidthUb
                       og = lambda(2);
                       %gotta double check if it's a wall
                       xTest = xs(okPoints);
                       yTest = ys(okPoints);
                        
                       %rOffset might need to be adjusted with more cases
                       rOffset = .02;
                       goodOnes = obj.findSailLinePoints([xx, yy], ...
                           obj.maxObjectPosnError + rOffset);
                       xTest = xTest(goodOnes)';
                       yTest = yTest(goodOnes)';
                       %estimate the length of the points
                       [x, y,~] = obj.findLengthandCenter(xTest,yTest);

                       % Calculate lambda
                       [lambda, ~] = obj.momentRoutine(x, y, numPts);
                       
                       %lamdaDiff might need to be adjusted with more cases
                       lambdaDiff = 5;
                       if (lambda(2) - og) > lambdaDiff
                           %disp("this is a wall")
                           continue
                       end
                       % It's a good sail and we can calculate the
                       % centroid and stuuff
                       centroid = [cx, cy];
                       % Compute angle of outward facing normal to line
                       th = atan2(2*Ixy,Iyy-Ixx)/2.0;
                       th = atan2(sin(th), cos(th));
                       break
                   end
               end
           end           
       end
   end
end
