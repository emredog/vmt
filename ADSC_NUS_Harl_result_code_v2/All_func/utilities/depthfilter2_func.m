function [ boxes ] = depthfilter2_func( boxes, depthimage )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
if ~isempty(boxes)
%     boxes = round(boxes);
    deleitems = [];
     [h w] = size(depthimage);
    
    for i = 1 : size(boxes,1)
        subdep = depthimage(2*boxes(i,2):2*boxes(i,4),...
            2*boxes(i,1):2*boxes(i,3));
        %         figure();imshow(subdep);
        
        left{1} = depthimage(2*boxes(i,2):2*boxes(i,4), max(2*boxes(i,1)-4,1):max(2*boxes(i,1)-2,1) );
        left{2} = depthimage(2*boxes(i,2):2*boxes(i,4), max(2*boxes(i,1)-1,1):max(2*boxes(i,1)+1,1) );
        left{3} = depthimage(2*boxes(i,2):2*boxes(i,4), max(2*boxes(i,1)+2,1):max(2*boxes(i,1)+4,1) );
        temp = [];
        for j = 1:3
            if ~isempty(left{j}(left{j}~=0) )
                temp = [temp median(left{j}(left{j}~=0))];
            end
        end
        
        left_value = max([temp]);

        temp = [];
        temp = subdep(subdep~=0);
        if ~isempty(temp)
            center_value = median(temp);
        else
            center_value = [];
        end
        
        right{1} = depthimage(2*boxes(i,2):2*boxes(i,4), min(2*boxes(i,3)-4,w):min(2*boxes(i,3)-2,w) );
        right{2} = depthimage(2*boxes(i,2):2*boxes(i,4), min(2*boxes(i,3)-1,w):min(2*boxes(i,3)+1,w) );
        right{3} = depthimage(2*boxes(i,2):2*boxes(i,4), min(2*boxes(i,3)+2,w):min(2*boxes(i,3)+4,w) );
%         right_value = max([median(right1(right1~=0)),median(right2(right2~=0)),median(right3(right3~=0))] );
        temp = [];
        for j = 1:3
            if ~isempty(right{j}(right{j}~=0) )
                temp = [temp median(right{j}(right{j}~=0))];
            end
        end
        
        right_value = max([temp]);

        if ~isempty(left_value) && ~isempty(center_value) && ~isempty(right_value)
            if ~(center_value<left_value && center_value<right_value )
                deleitems = [deleitems i];
            end
        end
    end
    boxes(deleitems,:)=[];
end

end

