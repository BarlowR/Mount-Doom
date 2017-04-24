function plotPath(sph)
    sph(2, :)
    pos = zeros(3, size(sph, 2))
    yaw = sph(2, :)
    distance = sph(1, :)
    %phi is roll
    %theta is pitch
    
    
    for i = 2:size(sph, 2)
        orientation = [cos(yaw(i)) -sin(yaw(i)) 0; sin(yaw(i)) cos(yaw(i)) 0; 0 0 1];
        yaw(i)
        post = distance(i).*orientation;
        pos(:, i) = post(:, 1) + pos(:, i-1);
    end
    
    plot3(pos(1, :), pos(2, :), pos(3, :))

end

