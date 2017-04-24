function Exercise1()
    %rosinit('10.0.75.2',11311, 'NodeHost','10.0.75.1')
        
    
    function setWheelV(vl,vr)
        msg = rosmessage(pub_vel);
        msg.Data = [vl vr];
        send(pub_vel,msg); 
    end
    
    
    pub_vel = rospublisher('/raw_vel');
    sub_scan = rossubscriber('/encoders');
    global running
    running = 1;
    tic
    
    wheelsub = receive(sub_scan);
    wheeldata = wheelsub.Data;

    initialEncoder = wheeldata;
    
    wheelbase = 0.24*1.07;
    % Define the gradient vector
    Gradient = @(x) [x(2)-2*x(1)-2; x(1)-2*x(2)-2];
    % set initial vals
    x = [4;1];
    ang = 0;
    dist=0;
    % Define the initial step-size
    lambda = 1/16;
    % Define the step-size multiplier
    delta = 1.2;
    time = 0;
    stop = 30;
    desiredDistance = 0;


    while(norm(Gradient(x)) > 0.01 && running == 1)  % run until within 0.01 of max
        oldx = x;
        % Compute the next point
        x = x + lambda.*Gradient(x);
        % Change the step-size
        lambda = lambda.*delta;

        desiredTravel = x - oldx; %determine what motion to take %Distance to move in next step
        
        %Works in theory, all units in radians and meters
        desiredAngle = atan2(desiredTravel(2), desiredTravel(1)); %compute angle from desiredTravel, 
        desiredDistance = desiredDistance + sqrt(desiredTravel(1)^2 + desiredTravel(2)^2);%compute distance from desiredTravel
        %Works in theory % + 1
        ang
        desiredAngle
        while ((ang > desiredAngle + .1 || ang < desiredAngle - .1) && running == 1) %rotate until within .01 radians of desired angle
            
            wheelsub = receive(sub_scan);
            wheeldata = wheelsub.Data;
        
            %rotate in desired direction
            RotVel = clip((ang - desiredAngle)/20, -0.19, 0.19)
            setWheelV(RotVel, -RotVel);
            
            %compute new angle and distance
            wheelsub = receive(sub_scan);
            wheeldata = wheelsub.Data;
            time = toc();
            distancel = distanceCalc(wheeldata(1), initialEncoder(1)); %write distance function
            distancer = distanceCalc(wheeldata(2), initialEncoder(2));
            ang = mod((distancer-distancel)/(wheelbase), 2*pi)
        end
        setWheelV(0,0)
        
        ang
        
        dist
        desiredDistance
        
        pause(1)
        while ((dist < desiredDistance - .1) && running == 1) %rotate until within .01 radians of desired angle
            
            wheelsub = receive(sub_scan);
            wheeldata = wheelsub.Data;
        
            %rotate in desired direction
            moveVel = clip((desiredDistance - dist)/4, -0.19, 0.19);
            setWheelV(moveVel, moveVel);
            
            %compute new angle and distance
            wheelsub = receive(sub_scan);
            wheeldata = wheelsub.Data;
            time = toc();
            distancel = distanceCalc(wheeldata(1), initialEncoder(1)); %write distance function
            distancer = distanceCalc(wheeldata(2), initialEncoder(2));
            desiredDistance
            dist = (distancel+distancer )/ 2
        end
        setWheelV(0,0)
        
        
        if(time > stop)
            running = 0;
        end
    end
end

function [val] = clip(valInit, rangeLo, rangeHi)
    if valInit > rangeHi
        val = rangeHi;
    elseif valInit < rangeLo
        val = rangeLo;
    else
        val = valInit;
    end
end

function [dist] = distanceCalc(encoderValue, initialValue)
    dist = (encoderValue - initialValue)  * 3.28;
end
    