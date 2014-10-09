function [ discusstions ] = Targets2discussions( targets , foldername, filter_length )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

discusstions = [];
di = 1;
if length(targets)>1
    %load depth value
    for i = 1:length(targets)
        targets{i}.data = round(targets{i}.data);
        for j = 1:size(targets{i}.data,1)
            k = targets{i}.data(j,5);
            depthfilename = sprintf('DepthMappingResults/%s/%s_%06d.png', foldername,foldername,k);
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
    
    for i = 1:length(targets)-1
        for j = i+1:length(targets)
            [inters IA IB] = intersect(targets{i}.data(:,5), targets{j}.data(:,5));
            if length(inters)>filter_length
                %fprintf('%d, %d\n',i,j);
                A = targets{i}.data(IA,:);
                B = targets{j}.data(IB,:);
                % rule 1: y coordinates of two boxes can not differ too
                % much
                YcorA = (A(:,2) + A(:,4))/2;
                YcorB = (B(:,2) + B(:,4))/2;
                r1 = (abs(YcorA-YcorB) - min(A(:,4) - A(:,2),B(:,4) - B(:,2))/2)<0;
                
                % rule 2: ratio between x distance and depth should not change
%                 XcorA = (A(:,1) + A(:,3))/2;
%                 XcorB = (B(:,1) + B(:,3))/2;
%                 XcorDis = abs(XcorA-XcorB);
% %                 DepAve = (A(:,6) + B(:,6))/2;
% %                 XDratio = XcorDiff./(DepAve+1);
% %                 
% %                 ratio1 = XDratio(1:end-1);
% %                 ratio2 = XDratio(2:end);
%                 XcorDiff = abs(XcorDis(2:end) - XcorDis(1:end-1)); 
%                 XcorDiff = medfilt1(XcorDiff,3);
%                 r2 = sum(abs(XcorDiff))/length(XcorDiff)<3.3;
%                  
                %This ratio should stay fixed. up/down <=0.1
                
                % rule 3: distance of depth value should not be too large
                % and should not change.
                dis_dep = abs(A(:,6) - B(:,6));
                dis_dep = medfilt1(dis_dep,3);
                r3_1 = dis_dep<2000;%normally 1000is enough, let's try 2000
                
                da = dis_dep(1:end-1);
                db = dis_dep(2:end);
                diff_dep = abs(db-da);
                 diff_dep = medfilt1(diff_dep,3);
                 r3_2 = sum(diff_dep)/length(diff_dep)<25;
%                 dis_dep2 = (A(:,6) - B(:,6));
%                 r3_2 = max(dis_dep)-min(dis_dep)<800;
                
                % rule 4: two targets can't be overlap
                Ax1 = A(:,1);
                Ay1 = A(:,2);
                Ax2 = A(:,3);
                Ay2 = A(:,4);

                Bx1 = B(:,1);
                By1 = B(:,2);
                Bx2 = B(:,3);
                By2 = B(:,4);

                Aarea = (Ax2-Ax1+1) .* (Ay2-Ay1+1);
                Barea = (Bx2-Bx1+1) .* (By2-By1+1);

                xx1 = max(Ax1, Bx1);
                yy1 = max(Ay1, By1);
                xx2 = min(Ax2, Bx2);
                yy2 = min(Ay2, By2);
                w = xx2-xx1+1;
                h = yy2-yy1+1;
                w = w.*(w>0);
                h = h.*(h>0);

                %o:overlap
                Aol = w .* h ./ Aarea;
                Bol = w .* h ./ Barea;

                oinBis = (Aol>0.2)&(Bol>0.2);
                oinBis2 = double(oinBis);
                oinBis = medfilt1(oinBis2,3);
                r4 = (sum(oinBis)/length(oinBis))<0.3;
                
                if ( sum(r1 .* r3_1)>filter_length ) * r3_2 * r4
                    discusstions{di}.targets_id = [i j];
                    discusstions{di}.data = [min(A(:,1),B(:,1)) min(A(:,2),B(:,2)) max(A(:,3),B(:,3)) max(A(:,4),B(:,4)) inters];
                    di = di+1;
                end
            end
        end
    end
end

end

