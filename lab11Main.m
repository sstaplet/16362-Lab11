%addpath(genpath('/Users/abbychen/Documents/senioryear/robot/_matlab_api'))
%set up robot
poseInit = [15*0.0254;9*0.0254;pi()/2.0];
initVec = [15*0.0254;9*0.0254;pi()/2.0];
initialPoseVec = [15*0.0254;9*0.0254;pi()/2.0] * (1+0.1*randn);
robotPose = pose(15*0.0254,9*0.0254,pi()/2.0);
rIF = robotIF(5,true, initialPoseVec);
rIF.startLaser();
doReadLoggedImages = false;
logOn = false; rstTraj = false;

% Set up lines
worldLineArray = worldModel.createXYAxes();

%Generate the map with the objects in it.
nPts = 10;
wallsShape = polyLineShape(worldLineArray); % create wall body from transposed lines
rIF.addObjects(wallsShape);
pause(2)
npts = 10;
pts1 = [worldLineArray(1:2,2) worldLineArray(1:2,2)];
pts2 = [worldLineArray(1:2,1) worldLineArray(1:2,3)];

%start mapping plots
figure(2);
ph1 = plot(worldLineArray(1,:),worldLineArray(2,:),'b');
axis([-.2 1.4 0 1.2])
hold on
bodyPts = robotModel.bodyGraph();

worldBodyPts = robotPose.bToA()*bodyPts;
ph2 = plot(worldBodyPts(1,:),worldBodyPts(2,:),'k');

%setup for drawing points
lidarSkip = 10; % Up to 30 should work

ranges = rIF.laser.LatestMessage.Ranges();
rImage = pointCloud(ranges,lidarSkip,true);
len = size(rImage.xArray,2);
points = [rImage.xArray ; rImage.yArray ; ones(1,len)];
worldLidarPts = pose(initialPoseVec)*points;
ph3 = plot(worldLidarPts(1,:),worldLidarPts(2,:),'r.','MarkerSize',12);
handler = robotKeypressDriver(gcf);
hold off
pause(2)
latestPoseVec = robotPose;
first = true;

while true
    %if first iteration of while loop, set up odometry and range variables
    if first
        odomStuff = stateEstimator(rIF, initVec, .25, pts1, ...
            pts2, .2, .00001, .0005);
        tEnc = odomStuff.encoderStartTimeStamp;
        first = false;
        newRange = false;
    end
    
    %peek to see if there's a new laser image
    laserVal = rIF.laser.LatestMessage.Ranges();
    if laserVal ~= ranges
        ranges = laserVal;
        newRange = true;
    end
    
    %wait to get a new encoder reading
    while tEnc == odomStuff.encoderLastTimeStamp
        pause(.001)
    end
    %update encoder vals;
    tEnc = odomStuff.encoderLastTimeStamp;
    odomVec = pose(odomStuff.getVec);

    % Draw it
    if newRange
        rImage = pointCloud(ranges,lidarSkip,true);
        len = size(rImage.xArray,2);
        points = [rImage.xArray ; rImage.yArray ; ones(1,len)];
        latestPoseVec = odomStuff.processRangeImage(points, 200);
        newRange = false;
    else
        latestPoseVec = odomVec;
    end

    worldBodyPts = latestPoseVec*bodyPts;
    worldLidarPts = latestPoseVec*points;

    set(ph1, 'XData', worldLineArray(1,:), 'YData',worldLineArray(2,:));
    set(ph2, 'XData',worldBodyPts(1,:), 'YData',worldBodyPts(2,:));
    set(ph3, 'XData',worldLidarPts(1,:), 'YData',worldLidarPts(2,:));
    handler.drive(rIF, 1.2)
end

