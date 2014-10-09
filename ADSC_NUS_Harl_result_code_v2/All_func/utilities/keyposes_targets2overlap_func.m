function [ actions ] = keyposes_targets2overlap_func(  targets,keyposes, tho1, tho2  )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% tho1 for targets; tho2 for keyposes
if nargin < 4
    tho1 = 0.5;
    tho2 = 0.2;
end
    
    
actions = [];
aci = 1;
if ~isempty(targets) && ~isempty(keyposes) 
    for i = 1:length(targets)
        [inters IA IB] = intersect(targets{i}.data(:,5), keyposes(:,5));
        if ~isempty(inters)
            tx1 = targets{i}.data(IA,1);
            ty1 = targets{i}.data(IA,2);
            tx2 = targets{i}.data(IA,3);
            ty2 = targets{i}.data(IA,4);

            kx1 = keyposes(IB,1);
            ky1 = keyposes(IB,2);
            kx2 = keyposes(IB,3);
            ky2 = keyposes(IB,4);

            tarea = (tx2-tx1+1) .* (ty2-ty1+1);
            karea = (kx2-kx1+1) .* (ky2-ky1+1);

            xx1 = max(tx1, kx1);
            yy1 = max(ty1, ky1);
            xx2 = min(tx2, kx2);
            yy2 = min(ty2, ky2);
            w = xx2-xx1+1;
            h = yy2-yy1+1;
            w = w.*(w>0);
            h = h.*(h>0);

            %o:overlap
            tol = w .* h ./ tarea;
            kol = w .* h ./ karea;

            oindis = (tol>tho1)|(kol>tho2);
            overlapframes = targets{i}.data(IA(oindis),5);
            if ~isempty(overlapframes)
                diffs = overlapframes(2:end) - overlapframes(1:end-1);
                indis = find(diffs>10);

                if(~isempty(indis))
                    sequences = zeros(length(indis)+1,2);
                    sequences(1,1)=1;
                    sequences(1,2)=indis(1);

                    sequences(end,1)=indis(end)+1;
                    sequences(end,2)=length(overlapframes)-indis(end);
                else
                    sequences = [1 length(overlapframes)];
                end

                if(length(indis)>1)
                    for k = 2:length(indis)
                        sequences(k,1) = indis(k-1)+1;
                        sequences(k,2) = indis(k)-indis(k-1);
                    end
                end

                sequences((sequences(:,2)<2),:) = [];
                for j=1:size(sequences,1)
                    starti = overlapframes(sequences(j,1));
                    endi = overlapframes(sequences(j,1)+sequences(j,2)-1);
                    actions{aci} = Targetdata2sub( targets{i}.data, starti, endi);
                    aci = aci + 1;
                end
            end
        end
    end
end

end

