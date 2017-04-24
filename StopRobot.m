function StopRobot()
        pub_vel = rospublisher('/raw_vel');
        sub_scan = rossubscriber('/encoders');

        tic();
        vl = 0.0;
        vr = -0.0;
        msg = rosmessage(pub_vel);
        msg.Data = [vl vr];
        send(pub_vel,msg); 
        
        wheelsub = receive(sub_scan);
        wheeldata = wheelsub.Data;
        Encoder = wheeldata;
        currentAng = 0;
        wheelbase = 0.24*1.07;
        while (toc() < 5)
            
            wheelsub = receive(sub_scan);
            wheeldata = wheelsub.Data;
            

            distancel = distanceCalc(wheeldata(1), Encoder(1)); %write distance function
            distancer = distanceCalc(wheeldata(2), Encoder(2));
            currentAng = mod(currentAng + (distancer - distancel/wheelbase), 2*pi);
            usefulCurrentAng = currentAng - pi;
            Encoder = wheeldata;
        end
        
       function [dist] = distanceCalc(encoderValue, initialValue)
        dist = (encoderValue - initialValue) * 3.28 ;
       end
        
end
 

    
