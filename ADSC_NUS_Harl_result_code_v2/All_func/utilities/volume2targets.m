function [ targets ] = volume2targets( imagepath, filter_length )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

CurrentTargets = [];
PastTargets = [];
matfilenames = dir([imagepath '\*.mat']);
framenums = 1+length(matfilenames);


for i = 2:framenums
    disp(i);
    matfilename = [imagepath '\' matfilenames(i-1).name];
    %load the boxes: plot_boxes
    load(matfilename);
    
    boxes = [];
    for j = 1:15
        boxes = [boxes; plot_boxes{j}];
    end
    picked = nms(boxes,0.5);
    boxes = boxes(picked,:);
    
    CurrentTargets = boxes2targets( CurrentTargets,boxes,i );
    
    if ~isempty(CurrentTargets)
        del = [];
        for j = 1:length(CurrentTargets)
            if i - CurrentTargets{j}.data(end,5) > 5
                PastTargets{end + 1} = CurrentTargets{j};
                del = [del j];
            end
        end
        CurrentTargets(del) = [];
        
    end
end
%%%

alltargets = [CurrentTargets PastTargets];
counts = [];
for i = 1:length(alltargets)
if size(alltargets{i}.data,1) >filter_length
counts = [counts i];
end
end
% length(counts)

%match the targets
targets = alltargets(counts);
% targets{:}

end

