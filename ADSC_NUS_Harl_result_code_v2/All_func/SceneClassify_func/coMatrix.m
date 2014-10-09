function [ m ] = coMatrix( I )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

% Itemp = zeros(480,640);
% Igray = double(rgb2gray(I));
% Itemp(find(abs(Igray-29)<3)) = 1;
% Itemp(find(abs(Igray-76)<3)) = 2;
% Itemp(find(abs(Igray-128)<3)) = 3;
% Itemp(find(abs(Igray-150)<3)) = 4;
% I = Itemp;

    d = 8;
    I = uint8(I);
    [h w] = size(I);
    m = zeros(4,4);
    for col = 1:w
        for row = 1:d:h-d
            if(I(row,col)~=0 && I(row+d,col)~=0)
                m( I(row,col), I(row+d,col) ) = m( I(row,col), I(row+d,col) ) + 1;
            end
        end
    end

end

