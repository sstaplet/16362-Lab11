%addpath(genpath('/Users/abbychen/Documents/senioryear/robot/_matlab_api'))
poseInit = [0.2;0.2;-pi*.5];

robotPose = pose(0.2,0.2,-pi*.5);

rIF = robotIF(5,true, poseInit);
rIF.startLaser();
doReadLoggedImages = false;
logOn = false; rstTraj = false;

k = 0.25;
gain = 0.2;
errThresh = 0.00001;
gradThresh = 0.0004;

lineArray = worldModel.createThreeWalls();

theSystem = MrplSystem(rIF, true, false, poseInit, k, lineArray, gain, ...
    errThresh, gradThresh, true);

lab12_World_Setup;
while size(picks,2) ~= 0
    [pal, picks] = theSystem.closestPose(picks);
    theSystem.pickDropObject(pose(pal), true);
    [drop, drops] = theSystem.closestPose(drops);
    theSystem.pickDropObject(pose(drop), false);
end