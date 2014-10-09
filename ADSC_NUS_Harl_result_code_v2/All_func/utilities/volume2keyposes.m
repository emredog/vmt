function [ keyposes ] = volume2keyposes( imagepath, model_ids , thresholds, threshold2 )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%   drop bag keypose model is modellist[5-6]
if nargin<4
    usetho2 = 0;
else
    usetho2 = 1;
end

if nargin<3
    usetho = 0;
else
    usetho = 1;
end

matfilenames = dir([imagepath '\*.mat']);
framenums = 1+length(matfilenames);
keyposes = [];

for i = 2:framenums
%     disp(i);
    matfilename = [imagepath '\' matfilenames(i-1).name];
    %load the boxes: plot_boxes
    load(matfilename);
    
    boxes = [];
%     for j = 1:length(model_ids)
%         boxes = [boxes; plot_boxes{model_ids(j)}];
%     end
    
    if ~usetho
        for j = 1:length(model_ids)
            boxes = [boxes; plot_boxes{model_ids(j)}];
        end
    else
        for j = 1:length(model_ids)
            tempboxes = plot_boxes{model_ids(j)};
            if ~isempty(tempboxes)
                tempboxes = tempboxes(tempboxes(:,5)>thresholds(j),:);
            end
            boxes = [boxes; tempboxes];
        end
    end
    
    if usetho2 == 1 && ~isempty(boxes)
        boxes = boxes(boxes(:,5)<threshold2, :);
    end
    
    picked = nms(boxes,0.5);
    boxes = boxes(picked,:);
    if ~isempty(boxes)
        boxes=boxes(:,1:4);
        boxes(:,5) = i;
        keyposes = [keyposes; boxes];
    end
end

end

