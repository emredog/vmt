function [ targetdata ] = targetdatainterp( targetdata )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
if ~isempty(targetdata) && size(targetdata,1) >1 

%         for j = 1:4
%             targetdata(:,j) = medfilt1(targetdata(:,j),3);
%         end
%         showtargets_func( targetdata2, foldername)
%         plot(targetdata(:,4))
    x = targetdata(:,5);
    y = targetdata(:,1:4);
    xi = (targetdata(1,5):targetdata(end,5))';
    yi = interp1(x,y,xi);
    targetdata = [yi xi];
end

end

