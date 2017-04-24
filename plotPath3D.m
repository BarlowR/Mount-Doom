function [poscur] = plotPath3D(sph, or, prevpos)
    %sph(2, :)
    yaw = sph(2, end);
    distance = sph(1, end);
    gx = or(1, end);
    gy = or(2, end);
    gz = or(3, end);
    %phi is roll
    %theta is pitch
    
    
    %for i = 2:size(sph, 2)
    orientationYaw = yaw;
    orientationTheta = atan(-gx/sqrt(gy^2 + gz^2));
    orientationPhi = atan(gy/gz);


    poscur = (rotatez(orientationYaw)*rotatey(orientationTheta)*rotatex(orientationPhi)*[distance; 0; 0])+prevpos;
    %end
    plot3(poscur(1), poscur(2), poscur(3), 'bo')

    function[rotX] = rotatex(radians)
        rotX = [1, 0, 0; 0, cos(radians), -sin(radians); 0, sin(radians), cos(radians)];
    end

    function[rotY] = rotatey(radians)
        rotY = [cos(radians), 0, sin(radians); 0, 1, 0; -sin(radians), 0, cos(radians)];
    end
    function[rotZ] = rotatez(radians)
        rotZ = [cos(radians), -sin(radians), 0; sin(radians), cos(radians), 0; 0, 0, 1];
    end
    
end

