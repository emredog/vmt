function [  ] = normmap_run( data_path,foldername, frames )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
resdir='NormMapResults\'; 
if ~exist(resdir,'dir')
    mkdir(resdir);
end

outputpath = [resdir foldername];
if ~exist(outputpath,'dir')
    mkdir(outputpath);
end

s = regexp(data_path,'\\','split');

% depthlist    = dir([data_path,'\depth-*','jp2']);
nimages = length(frames);
for i = 1:nimages
    depthname = [data_path,'\',sprintf('depth-%06d.jp2',frames(i))];
    outputname = [outputpath '\'  sprintf('norm-%06d.jpg',frames(i))];
    commandline = ['All_func\NormMap_func\ComputeNormMap.exe ' depthname ' ' outputname];
%     disp(commandline);
    system(commandline);
end

end

