function [W_out] = affineMBTracker(img, tmp, rect, W_in, context)
    % img is a geyscale image
    % tmp is the template image
    % rect is the bounding box
    % W_in is the previous frame
    % context is the precomputed J and H^-1 matrices
    %
    % W_out should be 3 by 3 that contains the new affine warp matrix updated so
    % that it aligns the CURRENT frame with the TEMPLATE
    
    % Get the rectangle
    x = rect(1);
    y = rect(2);
    w = rect(3);
    h = rect(4);
    
    % Get the image
    T = im2double(tmp);
    I = im2double(img);
    
    % Compute the multiplier
    mult = context.Hinv * context.J';
    
    % Get the ranges
    Xrange = x:x+w-1;
    Yrange = y:y+h-1;
    
    % Initialize W_out
    W_out = W_in;
    
    % Iterate for a maximum of 100 times
    max_iter = 10;
    tol = 1e-3;
    for iter = 1 : max_iter
        % Compute I(W(x;p))
        Iw = warpH(I, W_in, size(I), 0);
        
        % Compute the change in p
        dp = mult * reshape(Iw(Yrange,Xrange) - T,[],1);
        
        % Compute Wdp
        Wdp = [1+dp(1),dp(3),dp(5); dp(2),1+dp(4),dp(6); 0,0,1];
        
        % Update W_out
        W_out = W_out * inv(Wdp);
        
        % Check if dp is small
        if norm(dp) < tol
            break; 
        end
    end

end
