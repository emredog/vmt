function [  dooractions, dooractionclasses  ] = dooractiondetection_run3 ( standtargets )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
dooractions = [];
dooractionclasses = [];
dai = 0;
if ~isempty(standtargets)
    targets = standtargets;
    for i =1:length(targets)
        targetdata = targets{i}.data;
        targetdata = targetdata( targetdata(:,1)<45,: );
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
            
            sequences((sequences(:,2)>30),:) = [];
            
            for j=1:size(sequences,1)
                starti = frames(sequences(j,1));
                endi = frames(sequences(j,1)+sequences(j,2)-1);
                if (starti - targetdata(1,5))<10 || (targetdata(end,5)-endi)<10
                    dai = dai+1;
                    dooractions{dai} = Targetdata2sub( targetdata, starti, endi);
                    dooractionclasses(dai) = 4;
                end
            end
        end
    end
end

end

