function [ talksphones ] = TalksphoneDetection_run(  data_path )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
filter_length = 30;
[ targets ] = volume2certaintargets( data_path, filter_length ,[2 3 4 12 13 14 15]);
% this part need be changed too, 
% not only the standpose, but also typekeyboard pose.
keypose_model_ids = [];
[ keyposes ] = volume2keyposes( data_path, keypose_model_ids );%%don't know the id yet
[ talksphones ] = TalksphoneDetection_func( targets ,keyposes);

end

