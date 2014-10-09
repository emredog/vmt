function feature = ExtractNFeatures( I )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
w = 640;
h = 480;

Itemp = zeros(h,w);
Igray = double(rgb2gray(I));
Itemp(find(abs(Igray-29)<3)) = 1;
Itemp(find(abs(Igray-76)<3)) = 2;
Itemp(find(abs(Igray-128)<3)) = 3;
Itemp(find(abs(Igray-150)<3)) = 4;

pos1 = find(Itemp == 1);
pos2 = find(Itemp == 2);
pos3 = find(Itemp == 3);
pos4 = find(Itemp == 4);

p(1) = length( pos1 );
p(2) = length( pos2 );
p(3) = length( pos3 );
p(4) = length( pos4 );

% 1 four different value ratio
feature(1) = p(1)/double(sum(p));
feature(2) = p(2)/double(sum(p));
feature(3) = p(3)/double(sum(p));
feature(4) = p(4)/double(sum(p));

% 2 four different value's position center 
row = rem(pos1-1,h)+1;
col = ceil(pos1/h);
feature(5) = (sum(row)/length(row))/h;
feature(6) = (sum(col)/length(col))/w;

row = rem(pos2-1,h)+1;
col = ceil(pos2/h);
feature(7) = (sum(row)/length(row))/h;
feature(8) = (sum(col)/length(col))/w;

row = rem(pos3-1,h)+1;
col = ceil(pos3/h);
feature(9) = (sum(row)/length(row))/h;
feature(10) = (sum(col)/length(col))/w;

row = rem(pos4-1,h)+1;
col = ceil(pos4/h);
feature(11) = (sum(row)/length(row))/h;
feature(12) = (sum(col)/length(col))/w;

% 3 Co-occurrence matrix feature

m = coMatrix(Itemp);
feature(13) = ( m(2,4) + m(4,2) )/1000;

m = m/sum(sum(m));
feature(14) = IDM(m);
feature(15) = entropy(m);

rightpart = Itemp(:,540:640);
leftpart = Itemp(:,1:100);
% 1 means right to left, in scene 5 
% 3 means left to right, in scene 3
feature(16) = sum(sum(leftpart == 1))/sum(sum(leftpart~=0));
feature(17) = sum(sum(rightpart == 1))/sum(sum(rightpart~=0));
feature(18) = sum(sum(leftpart == 3))/sum(sum(leftpart~=0));
feature(19) = sum(sum(rightpart == 3))/sum(sum(rightpart~=0));
feature(20) = sum(sum(Itemp == 3))/sum(sum(Itemp~=0));
feature(21) = sum(sum(Igray~=0))/(640*480);
feature(22) = sum(sum(Itemp == 1))/sum(sum(Itemp~=0));
end

