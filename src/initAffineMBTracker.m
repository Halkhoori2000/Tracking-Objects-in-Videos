function [affineMBContext] = initAffineMBTracker(img, rect)
    % img is a greyscale image with a bouunding box rect
    % affineMBContext is a Matlab structure that contains the Jacobian of the
    % affine warp with respect to the 6 affine warp parameters and the inverse
    % of the approximated Hessian matrix (J and H^-1)
    % Get the rectangle parameters
    x = rect(1);
    y = rect(2);
    w = rect(3);
    h = rect(4);

    % Get the ranges
    Xrange = x:x+w-1;
    Yrange = y:y+h-1;

    % Get the template
    T = im2double(img);

    % Compute the image gradient
    [Tx, Ty] = imgradientxy(T);
    J = zeros(size(Yrange,2) * size(Xrange,2), 6);

    % Fill J and flatten 
    i = 1;
    for y=Yrange
        for x=Xrange
            J(i, :) = [Tx(y,x) Ty(y,x)] * [x 0 y 0 1 0; 0 x 0 y 0 1];
            i = i + 1;
        end
    end

    % Compute H inverse 
    Hinv = inv(J' * J);

    % Store in the structure
    affineMBContext.J = J;
    affineMBContext.Hinv = Hinv;

    % dW/dp = [u 0 v 0 1 0; 0 u 0 v 0 1];
    % J = DT dW/dp
    % H = J' * J
end

