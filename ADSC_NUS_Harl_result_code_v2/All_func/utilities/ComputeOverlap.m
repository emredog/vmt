function [ Aol, Bol ] = ComputeOverlap( A, B )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
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

end

