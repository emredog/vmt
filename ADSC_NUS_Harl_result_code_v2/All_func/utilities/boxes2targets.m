function [ CurrentTargets ] = boxes2targets( CurrentTargets,boxes,frameid )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

  if ~isempty(boxes)
        x1 = boxes(:,1);
        y1 = boxes(:,2);
        x2 = boxes(:,3);
        y2 = boxes(:,4);
        % remove unused filters
        if ~isempty(x1)
            for k = 1:length(x1)
                if ~isempty(CurrentTargets)
                    flag = 1;
                    for i = 1:length(CurrentTargets)
                        if compareBox2Target(CurrentTargets{i},x1(k),y1(k),x2(k),y2(k)) && flag
                            if CurrentTargets{i}.data(end,5) ~= frameid
                                CurrentTargets{i}.data(end+1,:) = [x1(k),y1(k),x2(k),y2(k),frameid];
                            end
                            flag = 0;
                        end
                    end
                    if flag
                        CurrentTargets{end+1} = newTarget(x1(k),y1(k),x2(k),y2(k),frameid);
                    end
                else
                    CurrentTargets{1} = newTarget(x1(k),y1(k),x2(k),y2(k),frameid);
                end
            end
        end
  end

end

