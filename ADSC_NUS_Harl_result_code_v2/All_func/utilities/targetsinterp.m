function [ targets ] = targetsinterp( targets )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
if ~isempty(targets)
    for i = 1:length(targets)
        if size(targets{i}.data,1) >1
            targetdata = targets{i}.data;
            for j = 1:4
                targetdata(:,j) = medfilt1(targetdata(:,j),3);
            end
    %         showtargets_func( targetdata2, foldername)
    %         plot(targetdata(:,4))

            x = targetdata(:,5);
            y = targetdata(:,1:4);
            xi = (targetdata(1,5):targetdata(end,5))';
            yi = interp1(x,y,xi);
            targetdata2 = [yi xi];
            targets{i}.data = targetdata2;
        end
    end
end
end

