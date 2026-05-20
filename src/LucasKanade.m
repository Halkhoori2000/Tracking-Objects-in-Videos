function [u, v] = LucasKanade(It, It1, rect)
    % Extract the rectangle parameters
    x = rect(1);
    y = rect(2);
    w = rect(3);
    h = rect(4);
    
    dp = [1,1];
    tol = 1e-2;
    
    % Initialize warp parameters 
    u = 0;
    v = 0;
    
    % Get the range 
    Xrange = x:min(size(It,2),x+w-1);
    Yrange = y:min(size(It,1),y+h-1);
    
    % Get the input image
    I = im2double(It1);
    
    % Get the template image
    T = im2double(It);
    
    % The jacobian matrix is identity matrix
    dWdp = [1 0; 0 1];
    
    % Compute the gradients
    [Ix,Iy] = imgradientxy(I);
    
    while norm(dp) > tol
        % Compute the warped coordinates
        [Xp, Yp] = meshgrid(Xrange+u, Yrange+v);

        % Compute DI
        DI = [reshape(interp2(Ix,Xp,Yp),[],1), reshape(interp2(Iy,Xp,Yp),[],1)];

        % Compute A = Sum[DI dW/dp]
        A = DI * dWdp;
        
        % Compute b = T(x) - I(W(x;p))
        b = reshape(T(Yrange, Xrange) - interp2(I,Xp,Yp),[],1);
        
        % Check norm of b
        if norm(b) < tol || isnan(norm(b)) 
            break 
        end 
        %fprintf(1, '%g\n', norm(b));
            
        % Using A and B compute the dp 
        dp = (A'*A)\(A'*b);
        
        % Update warp parameters (u,v)
        u = u + dp(1);
        v = v + dp(2);
 
    end
end 