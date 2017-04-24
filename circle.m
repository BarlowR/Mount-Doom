function circle()
    %rosinit('10.0.75.2',11311, 'NodeHost','10.0.75.1')
    function setWheelV(vl,vr)
        msg = rosmessage(pub_vel);
        msg.Data = [vl vr]*1.03;
        send(pub_vel,msg); 
    end

    pub_vel = rospublisher('/raw_vel');
    sub_scan = rossubscriber('/encoders');

    data = [0; 0]
    function processEncoders(sub, msg)
    end
    function [vl,vr] = calculateV(vLinear, vAngular)
              %units are m/s and radians/sec
        d = 0.24*1.07; %we need to experimentally detrmine this

        vl = vLinear + (vAngular*d)/2;
        vr = vLinear - (vAngular*d)/2;
        
        
    end

    global t
    syms t
    f = .1;
    a = .4;
    l = .4;
    stop = pi/f;
    disp(stop)
    r = symfun([-2*a*((l-cos(f*t))*cos(f*t)+(1-l)) 2*a*(l-cos(f*t))*sin(f*t)],t);
    v = norm(diff(r));
    T = diff(r)/v;
    K = norm(diff(T))/v;
    w = v*K;
%   
    
    wheelbase = 0.24*1.07;

    
    pub_vel = rospublisher('/raw_vel');
    sub_scan = rossubscriber('/encoders', @processEncoders);
    global running
    running = 1;
    tic
    while(running)
        time = toc();
        v_t = double(v(time));
        w_t = double(w(time));
        [vl,vr] = calculateV(v_t,w_t);
        setWheelV(vl,vr);
        if(time > stop)
            running = 0;
        end
        wheelsub = receive(sub_scan);
        wheeldata = wheelsub.Data;
        data = [(wheeldata(1) + wheeldata(2))/2, data(1, :); mod((wheeldata(2)-wheeldata(1))/(wheelbase), 2*pi), data(2, :)]
    end
    for i = 1:100
        setWheelV(0,0)
    end
    plotPath(data)
end