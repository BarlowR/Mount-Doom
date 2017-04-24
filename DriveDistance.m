function DriveDistance()
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
    stop = 30;
    tic
    
    wheelsub = receive(sub_scan);
    wheeldata = wheelsub.Data;

    Encoder = wheeldata;
    
    wheelbase = 0.24*1.07;
    % Define the gradient vector
    
Gradient = @(x) [x(2)-2*x(1)-2; x(1)-2*x(2)-2];
    % set initial vals
    x = [4;1];
    currentAng = pi/2;
    dist=0;
    % Define the initial step-size
    lambda = 1/16;
    % Define the step-size multiplier
    delta = 1.2;

while(norm(Gradient(x)) > 0.01)  % run until within 0.01 of max
        oldx = x;
        % Compute the next point
        x = x + lambda.*Gradient(x);
        % Change the step-size
        lambda = lambda.*delta;
        
        
        desiredTravel = x - oldx
        
        desiredAngle = atan2(desiredTravel(2), desiredTravel(1))
        usefulcurrentAng = 0
        desiredDistance = sqrt(desiredTravel(2)^2+desiredTravel(1)^2)
        
        setWheelV(-0.025, 0.025)
        while ((usefulcurrentAng - desiredAngle < -0.1 || usefulcurrentAng - desiredAngle > 0.1) && running == 1)
            
            wheelV = (desiredAngle - usefulcurrentAng)/ (16*pi);
            setWheelV(-wheelV, wheelV);
            desiredAngle
            
            wheelsub = receive(sub_scan);
            wheeldata = wheelsub.Data;
        
            distancel = distanceCalc(wheeldata(1), Encoder(1)); 
            distancer = distanceCalc(wheeldata(2), Encoder(2));
            currentAng = mod(currentAng + (distancer - distancel/wheelbase), 2*pi);
            usefulCurrentAng = currentAng - pi;
            Encoder = wheeldata;
            
            pause(0.1)
        end
        setWheelV(0, 0)    
        usefulcurrentAng
        pause(1)
        setWheelV(0.1, 0.1)
        while (dist < desiredDistance -0.01 && running == 1)
            
            wheelsub = receive(sub_scan);
            wheeldata = wheelsub.Data;
            
            distancel = distanceCalc(wheeldata(1), Encoder(1)); 
            distancer = distanceCalc(wheeldata(2), Encoder(2));
            dist = (distancel + distancer) / 2;
            
        end
        setWheelV(0, 0)
        dist
        
        if (toc() > stop)
            running = 0
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
    

