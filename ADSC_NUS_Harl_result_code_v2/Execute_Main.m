function [ ] = Execute_Main(folderRootPath )
%EXECUTE_MA�N Summary of this function goes here
%   Detailed explanation goes here

entryList = dir(folderRootPath);
isub = [entryList(:).isdir]; %# returns logical vector

folderList = {entryList(isub).name}';
folderList(ismember(folderList,{'.','..','.DS_Store','._.DS_Store'})) = [];

for f=1:length(folderList)
    Main_func_20140420([folderRootPath '/' folderList{f}]);
end
end

