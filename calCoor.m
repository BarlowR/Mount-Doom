function [sph, cart] = calcCoor(velr, vell, timeElap)
    Vang = (velr-vell)/2
    Vfor = (velr+vell)/2
    Dang = Vang*timeElap
    Dfor = Vfor*timeElap
    
    sph = [Dang ; Dfor]
    x = Dfor * cos(Dang)
    y = Dfor * sin(Dang)
    cart = [x; y]
    

end
