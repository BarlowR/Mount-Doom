clf;
clear;
unitConv = 3.281; %meters to feet
pub = rospublisher('/raw_vel');
sub = rossubscriber('/encoders');
accsub = rossubscriber('/accel');

sph = [0; 0];
or = [0; 0; 0];
prevpos = [0;0;0];


d=.25*unitConv;
Ry = @(th) [cos(th) 0 -sin(th); 0 1 0; sin(th) 0 cos(th)];
robZfunc = @(accel) [[0 0; 0 0; 0 0] accel]*[0; 0; 1];
Gradient = @(robZ) [0; 0; 1] - (robZ);
robXfunc = @(grad) [grad(1); grad(2); norm(grad)^2];
robYfunc = @(robZ, robX) cross(robZ, robX);
init_th = 0.336; %robot angle offset in degrees

% make sure when finding the difference between these axes and the robot axes,
% convert these into robot coordinate system


%Gradient = @(x) [(x(2)-2*x(1)-2); (x(1)-2*x(2)-2)];
%Tangent = @(x) [cos(pi/2) sin(pi/2); -sin(pi/2) cos(pi/2)]*[(x(2)-2*x(1)-2); (x(1)-2*x(2)-2)]
% set initial vals
% Define the initial step-size
lambda = .5;
stepsize = .4;
% Define the step-size multiplier
%delta = 1.2;
time = 0;
stoptime =50;
stop = 0;
movmult = 1;
maxspd = .1;
sharpness = 4; %How much the robot cares about turning to the right angle before moving.  If set too high it drives a really curvy path, if set too low it cant turn fast enough to get to its target 
%^ 1 tends to do the job fine, 2 is too much with maxspd of .1
%desiredDistance = 0;

strtmsg = rosmessage(pub);
stopmsg = rosmessage(pub);

stopmsg.Data = [0, 0];

tic

%send(pub, strtmsg)
%wheeldata = receive(sub);
wheeldata = receive(sub);
data_old = wheeldata.Data * unitConv;
hold on;
points = [4 -8 4 -6; 1 -5 1 -6];
ang = pi/2;
j = 1;
x = [0;0];
% while toc < stoptime
%     accdata = receive(accsub);
%     robZ = robZfunc(accdata.Data);
%     grad = Gradient(robZ);
%     robXpre = robXfunc(grad);
%     robYpre = robYfunc(robZ, robXpre);
%     robX = robXpre/norm(robXpre);
%     robY = robYpre/norm(robYpre);
%     norm(grad)
%     des_rob_Axes = [robX robY robZ];
%     
% end
accdata = receive(accsub);
robZ = robZfunc(Ry(init_th)*accdata.Data);
grad = Gradient(robZ);
while norm(grad) > .03
    accdata = receive(accsub);
    robZ = robZfunc(Ry(init_th)*accdata.Data);
    grad = Gradient(robZ);
    
    or = [or, robZ];
    
    %robXpre = robXfunc(grad);
    %robYpre = robYfunc(robZ, robXpre);
    %robX = robXpre/norm(robXpre);
    %robY = robYpre/norm(robYpre);
    %norm(grad)
    %des_rob_Axes = [robX robY robZ];

    %v = norm(dr)
    desTrav = lambda.*grad;
    desTrav = desTrav/norm(desTrav);
    desAng = mod(atan2(desTrav(2), desTrav(1)),2*pi);
    %lambda = lambda.*delta;
    wheeldata = receive(sub);
    data_new = wheeldata.Data * unitConv;
    dpl = data_new(1) - data_old(1);
    dpr = data_new(2) - data_old(2);
    dp = (dpl + dpr)/2;
    dang=(dpr-dpl)/d;
    
    sph = [sph(1) dp; sph(2) dang];
    dx=dp*cos(ang);
    dy=dp*sin(ang);
    %dz=
    ang = ang+dang;
    x = x+[dx;dy];
    v = norm(desTrav);

    angDiff = ang - desAng;
    if angDiff > pi
        angDiff = angDiff - 2*pi;
    elseif angDiff < -pi
        angDiff = angDiff + 2*pi;
    end

    w = -angDiff*sharpness;
    Vs = magclip2(v-((w*d)/2), v+((w*d)/2), maxspd);
    VL = Vs(1);
    VR = Vs(2);
    strtmsg.Data = [VL, VR];
    send(pub, strtmsg)
    if toc>stoptime
        send(pub, stopmsg)
        break
    end
    data_old = data_new;
    %plot(x(1), x(2), 'bo')
    prevpos = plotPath3D(sph, or, prevpos)
end

send(pub, stopmsg)