function [ subfragment ] = Targetdata2sub( targetdata, a, b)
%UNTITLED Summary of this function goes here
%   Get a sub fragment of targetdata and return
%   a,b indicates start and end frame values
indis = (targetdata(:,5)>=a) & (targetdata(:,5)<=b);
subfragment = targetdata(indis,1:5);

end

