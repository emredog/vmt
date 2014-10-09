function [ tapkeyboards ] = tapkeyboarddetect_run( data_path )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
filter_length = 100;
[ tapkeyboards ] = volume2certaintargets( data_path, filter_length, [6 7 8 9] );

end

