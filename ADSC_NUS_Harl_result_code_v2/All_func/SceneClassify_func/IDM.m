function re = IDM( m )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
re = 0;
[h w] = size(m);
for i = 1:h
    for j = 1:w
        re = re+m(i,j)/(1+(i-j)^2);
    end
end

end

