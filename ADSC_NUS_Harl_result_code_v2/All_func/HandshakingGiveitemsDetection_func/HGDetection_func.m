function [ HandshakingGiveitems ] = HGDetection_func( targets ,keyposes)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
HandshakingGiveitems = [];
hgi = 0;
if length(targets)>1 && ~isempty(keyposes)
    for i = 1:length(targets)-1
        for j = i+1:length(targets)
            [inters IA IB] = intersect(targets{i}.data(:,5), targets{j}.data(:,5));
            if ~isempty(inters)
                [inters2 Iab IC] = intersect(targets{i}.data(IA,5),keyposes(:,5));
                if ~isempty(inters2)
                    A_ab = targets{i}.data(IA,:);
                    B_ab = targets{j}.data(IB,:);
                    
                    A = A_ab(Iab,:);
                    B = B_ab(Iab,:);
                    C = keyposes(IC,:);
                    
                    % rule 1: y coordinates of two humans can not differ too
                    % much
                    YcorA = (A(:,2) + A(:,4))/2;
                    YcorB = (B(:,2) + B(:,4))/2;
                    r1_t = (abs(YcorA-YcorB) - min(A(:,4) - A(:,2),B(:,4) - B(:,2))/2)<0;
                    r1 = double(sum(r1_t))/length(r1_t)>0.7;
                    % rule 2: two humans can't be overlap
                    [ Aol, Bol ] = ComputeOverlap( A, B );
                    oinBis = (Aol>0.2)&(Bol>0.2);
                    oinBis = medfilt1(oinBis,3);
                    r2 = (sum(oinBis)/length(oinBis))<0.3;
                    
                    % rule 3: handkeypose are overlape with both of humans
%                     [ Aol, ACol ] = ComputeOverlap( A, C );
%                     [ Aol, BCol ] = ComputeOverlap( B, C );
%                     oinBis = (ACol>0.2)&(BCol>0.2);
%                     r3 = medfilt1(oinBis,3);
%                     r3 = (sum(oinBis)/length(oinBis))>0.8;
                    % rule 3:x distance between keypose and human boxes <20
                    r3_1 = abs( A(:,3)  - C(:,1) )<=100;
                    r3_2 = abs( B(:,1)  - C(:,3) )<=100;
                    r3 = r3_1 & r3_2;
                    % rule 4: the handkeypose is in between two humans
                    XcorA = (A(:,1) + A(:,3))/2;
                    XcorB = (B(:,1) + B(:,3))/2;
                    XcorC = (C(:,1) + C(:,3))/2;
                    
                    r4 = (XcorC<max(XcorA, XcorB) & XcorC>min(XcorA, XcorB));
%                     r4 = (sum(r4indi)/length(r4indi))>0.8;
                    % rule 5: the Y coordinate of handkeypose should
                    % between two humans'
                    YcorA = (A(:,2) + A(:,4))/2;
                    YcorB = (B(:,2) + B(:,4))/2;
                    r5_1 = min( A(:,2), B(:,2) );
                    r5_2 = max( A(:,4), B(:,4) );
                    r5 = (C(:,2) > r5_1) & (C(:,4) < r5_2);

                    if(r1&r2)
                        frames = A((r3&r4&r5),5);
                        
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
                                for i = 2:length(indis)
                                    sequences(i,1) = indis(i-1)+1;
                                    sequences(i,2) = indis(i)-indis(i-1);
                                end
                            end

                            for k=1:size(sequences,1)
                                hgi = hgi+1;
                                starti = frames(sequences(k,1));
                                endi = frames(sequences(k,1)+sequences(k,2)-1);
                                indis = (A(:,5)>=starti) & (A(:,5)<=endi);
%                                 talksphones{tpi} = Targetdata2sub( targets{i}.data, starti, endi);
                                HandshakingGiveitems{hgi} = [min(A(indis,1),B(indis,1)) min(A(indis,2),B(indis,2)) ...
                                    max(A(indis,3),B(indis,3)) max(A(indis,4),B(indis,4)) A(indis,5)];
                            end
                        end
                        
                    end
                end
            end
        end
    end
end

end

