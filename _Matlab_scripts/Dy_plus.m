%==========================================================================

function dy_plus=Dy_plus(u, h)

% Author: Prashant Athavale
% Date 11/19/2011
% This function takes a grayscale image variable u and gives its derivative
% The derivative is the forward x-derivative,
% i.e. we try to approximate the derivative with (u(y+h)-u(y))/h
% but note that the y-axis is taken going to the right 
% the origin is at the top left corner.
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
    dy_plus=zeros([M, N, P]);
    for i=1:M
        for j=1:N-1
            dy_plus(i, j, :)=(u(i, j+1, :)-u(i, j, :))/h;
        end
    end
end
end % end of the function Dy_plus

%==========================================================================