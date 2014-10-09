function [ targets ] = targetslink( targets, linkrange, overlap_threshold, low_threshold)
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
% function for link the targets
% linkrange = 40;
if nargin < 4
    low_threshold = 0.6;
end
if nargin < 3
    overlap_threshold = 0.7;
end
if nargin < 2
    linkrange = 40;
end
% find relations between targets
len = length(targets);
relations = zeros(len,len);
distances = zeros(len,len);
overlaps = zeros(len,len);
for i = 1:len-1
    for j = i+1:len
        [inters] = intersect(targets{i}.data(:,5), targets{j}.data(:,5));
        if isempty(inters)
            
            c1 = abs(targets{i}.data(1,5) - targets{j}.data(end,5));
            c2 = abs(targets{i}.data(end,5) - targets{j}.data(1,5));
            [distance, indi] = min([c1, c2]);
            
            if indi==1
                box1 = targets{i}.data(1,1:4);
                box2 = targets{j}.data(end,1:4);
            else
                box1 = targets{i}.data(end,1:4);
                box2 = targets{j}.data(1,1:4);
            end
            
            [ Aol, Bol ] = ComputeOverlap( box1, box2 );
            
            if distance < linkrange && ( (Aol>overlap_threshold && Bol>low_threshold)...
                    || (Bol>overlap_threshold && Aol>low_threshold ) || ...
                    (Aol>0.5 && Bol>0.9)...
                    || (Bol>0.5 && Aol>0.9 ))
                if indi == 1
                    relations(j,i) = 1;
                else
                    relations(i,j) = 1;
                end
                distances(i,j) = distance;
                overlaps(i,j) = max(Aol,Bol);
            end
        end
    end
end
%filter multi links
for i = 1:len
    
    if sum(relations(i,:))>1
        relations(i,:) = (distances(i,:) == min(distances(i,:)));
        if sum(relations(i,:))>1
            overlaps(i,:) = overlaps(i,:).*relations(i,:);
            relations(i,:) = (overlaps(i,:) == max(overlaps(i,:)));
            if sum(relations(i,:))>1
                pos = find(relations(i,:),1);
                relations(i,:) = zeros(1,len);
                relations(i,pos) = 1;
            end
        end
    end
    
    if sum(relations(:,i))>1
        relations(:,i) = (distances(:,i) == min(distances(:,i)));
        if sum(relations(:,i))>1
            overlaps(:,i) = overlaps(:,i).*relations(:,i);
            relations(:,i) = (overlaps(:,i) == max(overlaps(:,i)));
            if sum(relations(:,i))>1
                pos = find(relations(:,i),1);
                relations(:,i) = zeros(1,len);
                relations(pos,i) = 1;
            end
        end
    end
    
end

%link the targets
if ~isempty(find(relations, 1))
    links = [(rem(find(relations)-1,len)+1) ceil(find(relations) / len)];
    [temp,index] = sort(links(:,2));
    links = links(index,:);
    
    deli = [];
    for i = 1:size(links,1)
        targets{links(i,2)}.data = [targets{links(i,1)}.data ; targets{links(i,2)}.data];
        deli = [deli links(i,1)];
    end
    
    targets(deli) = [];
    [ targets ] = targetsinterp( targets );
end

end

