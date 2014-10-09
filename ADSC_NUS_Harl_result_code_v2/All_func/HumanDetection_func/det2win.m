function [det_win] = det2win(det, scales, sbin, padx, pady, filter_sizes, max_image_size)

% det = det{1}; % for test purpose only later will be looped

det_win = [];
num_of_filters = length(det); 
for i = 1 : num_of_filters
    det_win{i} = [];
end


for i = 1 : length(det) %per filter
    for j = 1 : size(det{i},1) %per box

        x1 = (det{i}(j, 2) - padx - 1) * sbin / scales(det{i}(j,4)); 
        y1 = (det{i}(j, 3) - pady - 1) * sbin / scales(det{i}(j,4)); 
        x2 = x1 + filter_sizes{i}(2) * sbin / scales(det{i}(j,4)) - 1;
        y2 = y1 +  filter_sizes{i}(1) * sbin / scales(det{i}(j,4)) - 1;

        x1 = max(x1, 1);
        x2 = min(max_image_size(2),x2);
        y1 = max(y1, 1);
        y2 = min(max_image_size(1),y2);

        s = det{i}(j,5);

        det_win{i} = [det_win{i};[x1 y1 x2 y2 s]];
    end
end


