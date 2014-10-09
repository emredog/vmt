function [  dooractions, dooractionclasses  ] = dooractiondetection_run2 ( foldername, standtargets )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
dooractions = [];
dooractionclasses = [];
dai = 0;
if ~isempty(standtargets)
    path = ['DepthMappingResults\' foldername];
    imagelist = dir([path , '\*.png']);
    image = imread([path '\' imagelist(1).name]);
    subleft = image(:,30:60);
    subright = image(:,560:590);
    depthleft = median(subleft(subleft~=0));
    depthright = median(subright(subright~=0));

    targets = standtargets;
    % load depth valuse
    for i = 1:length(targets)
        for j = 1:size(targets{i}.data,1)
            k = targets{i}.data(j,5);
            depthfilename = sprintf('DepthMappingResults\\%s\\%s_%06d.png', foldername,foldername,k);
            depthimage = imread(depthfilename);
            subdp = depthimage(2*targets{i}.data(j,2):2*targets{i}.data(j,4), 2*targets{i}.data(j,1):2*targets{i}.data(j,3));
            %figure(1);imshow(subdp);
            if ~isempty(subdp(subdp~=0))
                targets{i}.data(j,6) = median(subdp(subdp~=0));
            else
                targets{i}.data(j,6) = 0;
            end
            %targets{i}.data(j,6) = depthimage()
        end
    end

    for i =1:length(targets)
        targetdata = targets{i}.data;
        targetdata = targetdata( targetdata(:,6)>(min(depthleft,depthright)) ...
            & targetdata(:,6)<(max(depthleft,depthright)+100),1:5 );
        frames = targetdata(:,5);
        if ~isempty(frames)
            %process the frames into fragments of targets
            diffs = frames(2:end) - frames(1:end-1);
            indis = find(diffs>10);

            if(~isempty(indis))
                sequences = zeros(length(indis)+1,2);
                sequences(1,1)=1;
                sequences(1,2)=indis(1);

                sequences(end,1)=indis(end)+1;
                sequences(end,2)=length(frames)-indis(end);
            else
                sequences = [1 length(frames)];
            end

            if(length(indis)>1)
                for k = 2:length(indis)
                    sequences(k,1) = indis(k-1)+1;
                    sequences(k,2) = indis(k)-indis(k-1);
                end
            end
            
            sequences((sequences(:,2)<3),:) = [];
            
            for j=1:size(sequences,1)
                starti = frames(sequences(j,1));
                endi = frames(sequences(j,1)+sequences(j,2)-1);
                dai = dai+1;
                dooractions{dai} = Targetdata2sub( targetdata, starti, endi);
                dooractionclasses(dai) = 4;
            end
        end
    end
end
end

