function [ re ] = compareBox2Target(target,x1,y1,x2,y2)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
tx1 = target.data(end,1);
ty1 = target.data(end,2);
tx2 = target.data(end,3);
ty2 = target.data(end,4);

xx1 = max(x1, target.data(end,1));
yy1 = max(y1, target.data(end,2));
xx2 = min(x2, target.data(end,3));
yy2 = min(y2, target.data(end,4));
w = xx2-xx1+1;
h = yy2-yy1+1;
if w > 0 && h > 0
    o = max( w * h / ((x2-x1+1) * (y2-y1+1)), w * h / ((tx2-tx1+1) * (ty2-ty1+1)) );
        if o > 0.5
            re = 1;
        else
            re = 0;
        end
else
    re = 0;
end
end

