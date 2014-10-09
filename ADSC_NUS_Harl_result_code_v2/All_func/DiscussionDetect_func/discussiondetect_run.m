function [ discusstions ] = discussiondetect_run( data_path, foldername )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
filter_length = 30;
[ targets ] = volume2certaintargets( data_path, filter_length ,[1 2 3]);
[ discusstions ] = Targets2discussions( targets ,foldername,  filter_length );
% showtargets_func(discusstions{1}.data, foldername);
if ~isempty(discusstions)
    for i = 1:length(discusstions)
        discusstions{i} = discusstions{i}.data;
    end
end
end

