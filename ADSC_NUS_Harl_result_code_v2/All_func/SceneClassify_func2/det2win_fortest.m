function [det_win] = det2win_fortest(det, scales, sbin, padx, pady, filter_size, max_image_size)

det = det{1}; % for test purpose only later will be looped

det_win = [];

for i = 1 : size(det,1)
    
    x1 = (det(i, 2) - padx - 1) * sbin / scales(det(i,4)); 
    y1 = (det(i, 3) - pady - 1) * sbin / scales(det(i,4)); 
    x2 = x1 + filter_size(2) * sbin / scales(det(i,4)) - 1;
    y2 = y1 +  filter_size(1) * sbin / scales(det(i,4)) - 1;
    
    x1 = max(x1, 1);
    x2 = min(max_image_size(2),x2);
    y1 = max(y1, 1);
    y2 = min(max_image_size(1),y2);
    
    s = det(i,5);
    
    det_win = [det_win;[x1 y1 x2 y2 s]];
    
end

