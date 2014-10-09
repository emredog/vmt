%%threshold_detections
function [det] = scores2det(scores,thresholds)

det_filters = [];
num_of_filters = length(scores{1}); 
for i = 1 : num_of_filters
    det_filters{i} = [];
end

for i = 1 : length(scores)  % per level
    for j = 1 : length(scores{i}) %per filter
        [tmpI] = find(scores{i}{j} > thresholds(j));
        [tmpY, tmpX] = ind2sub(size(scores{i}{j}), tmpI);
        [tmpL] = [i*ones(length(tmpI), 1)];
        [tmpS]= [scores{i}{j}(tmpI)];
        det_filters{j} = [det_filters{j};[tmpI tmpX tmpY tmpL tmpS]];
    end
end

det = det_filters;

end