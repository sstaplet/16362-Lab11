function bodyPts = bodyGraph()
    % return an array of points that can be used to plot robot
    % body in a window.
    % angle arrays

    rad = 0.06; % robot radius
    laser_rad = 0.04; % laser housing radius

    step = pi/20;
    cir = 0: step: 2.0*pi;
    % circle for laser
    lx = laser_rad*cos(cir);
    ly = laser_rad*sin(cir);
    
    % body with implicit line to laser circle
    bx = [rad*cos(cir) lx];
    by = [rad*sin(cir) ly];
    
    %create homogeneous points
    bodyPts = [bx ; by ; ones(1,size(bx,2))];
end