%==========================================================================

function dx_plus=Dx_plus(u, h)

% Author: Prashant Athavale
% Date 11/19/2011
% Please acknowledge my name if you use this code, thank you.

% This function takes a grayscale image variable u and gives its derivative
% The derivative is the forward x-derivative,
% i.e. we try to approximate the derivative with (u(x+h)-u(x))/h
% but note that the x-axis is taken going down, top left is the origin
% and the distance between the two pixels, h, is taken as 1-unit
% Some people like to take the distance h=1/M for square images where M is the dimension of the image.

%==========================================================================

if nargin==0;
    error('At least one input is needed');
elseif nargin>0
    if nargin==1
        h=1; % the distance between two pixels is defined as 1
    end
    [M, N, P]=size(u);

    if isa(u, 'double')==0
        u=double(u);
    end
    dx_plus=zeros([M, N, P]);
    for i=1:M-1
        for j=1:N
            dx_plus(i, j, :)=(u(i+1, j, :)-u(i, j, :))/h;
        end
    end
end

end % end of the function Dx_plus

%==========================================================================