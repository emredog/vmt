function [  ] = depthmapping_run( data_path, foldername )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
resdir='DepthMappingResults/'; 
if ~exist(resdir,'dir')
    mkdir(resdir);
end

outputpath = [resdir foldername];
if ~exist(outputpath,'dir')
    mkdir(outputpath);
end

depthlist    = dir([data_path,'/depth-*','jp2']);
nimages = length(depthlist);
for i = 1:nimages
    depthname = [data_path,'/',depthlist(i).name];
%     outputname = [resdir s{end} '_' depthlist(i).name(7:12) '.jpg'];
%     commandline = ['DepthMapping_func\DepthMapping.exe ' depthname ' ' outputname];
%     disp(commandline);
    outputname = [outputpath '/' foldername '_' depthlist(i).name(7:12) '.png'];
    commandline = ['All_func/DepthMapping_func/DepthMapping16bit.exe ' depthname ' ' outputname];
    system(commandline);
end
    
end

