function [ oratio ] = CompareTwoFrames( frameA, frameB )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% compare two frames's diff ratio.

d = abs(frameA - frameB);
bw = im2bw(d, 0.2);

[h w] = size(frameA);
diff_indis = find(bw==1);


if ~isempty(diff_indis)
    h_indis = rem((diff_indis-1),h)+1;
    w_indis = ceil(diff_indis/h);

    top = min(h_indis);
    bottom = max(h_indis);
    left = min(w_indis);
    right = max(w_indis);
    
    wd = right-left;
    hg = bottom-top;
    
    oratio = (wd*hg)/(w*h);
else
    oratio = 0;
end

end

