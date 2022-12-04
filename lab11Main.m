%addpath(genpath('/Users/abbychen/Documents/senioryear/robot/_matlab_api'))
%set up robot
poseInit = [0.6096;0.6096;pi()/2.0];

robotPose = pose(0.6096,0.6096,pi()/2.0);

rIF = robotIF(5,true, poseInit);
rIF.startLaser();
doReadLoggedImages = false;
logOn = false; rstTraj = false;

% Set up lines
lineArray = worldModel.createXYAxes();

%Generate the map with the objects in it.
nPts = 10;
wallsShape = polyLineShape(worldLineArray); % create wall body from transposed lines
rIF.addObjects(wallsShape);

%lines_p1 = [worldLineArray(1:2,2) worldLineArray(1:2,2)];
%lines_p2 = [worldLineArray(1:2,1) worldLineArray(1:2,3)];

k = 0.25;
gain = 0.2;
errThresh = 0.00001;
gradThresh = 0.0005;

mrpl = MrplSystem(rIF, true, false, poseInit, k, lineArray, gain, errThresh, gradThresh, true);

vmax = 0.2;

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
worldLidarPts = pose(poseInit)*points;
ph3 = plot(worldLidarPts(1,:),worldLidarPts(2,:),'r.','MarkerSize',12);
handler = robotKeypressDriver(gcf);
mrpl.trajectoryFollower.setLogInfo(ph1, ph2, ph3)
hold off
pause(2)
latestPoseVec = robotPose;


xf = 0.3048; yf = 0.9144; thf = pi()/2.0;
mrpl.executeTrajectoryToAbsPose(xf, yf, thf, vmax, 1.0);

xf = 0.9144; yf = 0.3048; thf = 0.0;
mrpl.executeTrajectoryToAbsPose(xf, yf, thf, vmax, 1.0);

xf = 0.6096; yf = 0.6096; thf = pi()/2.0;
mrpl.executeTrajectoryToAbsPose(xf, yf, thf, vmax, 1.0);

figure(3);
hold on;
plot(mrpl.ctrler.ref_traj(1,:), mrpl.ctrler.ref_traj(2,:), "b");
plot(mrpl.ctrler.act_traj(1,:), mrpl.ctrler.act_traj(2,:), "g");
