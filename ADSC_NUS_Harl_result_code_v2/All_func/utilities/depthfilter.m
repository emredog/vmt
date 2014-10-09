function [ boxes ] = depthfilter( boxes, foldername, frameid )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    depthfilename = sprintf('DepthMappingResults/%s/%s_%06d.png', foldername,foldername,frameid);
    depthimage = imread(depthfilename);
    delindi = [];
    boxes = round(boxes);
    for j = 1:size(boxes,1)
        subdp = depthimage(2*boxes(j,2):2*boxes(j,4), 2*boxes(j,1):2*boxes(j,3));
        if ~isempty(subdp(subdp~=0))
            depthvalue = median(subdp(subdp~=0));
        else
            depthvalue = 0;
        end
        width = 2*boxes(j,3) - 2*boxes(j,1);
        height = 2*boxes(j,4) - 2*boxes(j,2);
        if depthvalue~=0
            if depthvalue>33100
                if height>180 %very far
                    delindi = [delindi j];
                end
            end
%             if depthvalue>31000 && depthvalue<=33100
%                 if ( height>380 || height<150 )%( height>320 || height<200 )
%                     delindi = [delindi j];
%                 end
%             end
            if depthvalue>32000 && depthvalue<=33100
                if ( height>300 || height<130 )%( height>320 || height<200 )
                    delindi = [delindi j];
                end
            end
            if depthvalue>31000 && depthvalue<=32000
                if ( height>380 || height<200 )%( height>320 || height<200 )
                    delindi = [delindi j];
                end
            end
            if depthvalue>30000 && depthvalue<=31000
                if ( height<300 )
                    delindi = [delindi j];
                end
            end
            if depthvalue <= 30000
                if ( height<350 )
                    delindi = [delindi j];
                end
            end
        end
    end
    boxes(delindi,:) = [];
    %depthvalue
    %size 
    if ~isempty(boxes)
        [ boxes ] = depthfilter2_func( boxes, depthimage );
    end
end

