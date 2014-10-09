function [ re ] = sceneclassify_run2( data_path , ifs3)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% imagename = 'C:\Users\Pei.Yong\Downloads\wget-1.10.2b\d1-test\test\vid0180\gray-000001.jpg';
if nargin < 2
    ifs3 =0;
end
imagelist = dir([data_path '/*.jpg']);
imagename = [data_path '/' imagelist(1).name];
load S1.mat;
re1 = test_func3( model, imagename, sample_width, sample_height);
load S2.mat;
re2 = test_func3( model, imagename, sample_width, sample_height);
load S3.mat;
re3 = test_func3( model, imagename, sample_width, sample_height);

re = 0;

if ifs3 == 0
    if re1 == 1
        re = 1;
    else
        if re2 == 1
            re = 2;
        end
    end
else
    if re3 == 1
        re = 3;
    end
end

end

