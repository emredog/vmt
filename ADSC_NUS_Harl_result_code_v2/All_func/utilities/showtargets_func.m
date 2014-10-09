function [ ] = showtargets_func( targetdata, foldername, outpath)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%load foldernames_wholeset;
% fi = 14;
data_path = ['C:\Users\Pei.Yong\Downloads\wget-1.10.2b\d1-training-validation\training-validation\' foldername];
nimages = size(targetdata,1);%length(imagelist);

%outputpath = [resdir foldername(fi).name '_result'];
for i = 1:nimages
    ima = targetdata(i,5);
    if ima<10
        imagename = ['gray-00000' num2str(ima) '.jpg'];
    end
    if ima>9 && ima<100
        imagename = ['gray-0000' num2str(ima) '.jpg'];
    end
    if ima>99 && ima<1000
        imagename = ['gray-000' num2str(ima) '.jpg'];
    end
    
    im_test = imread([data_path, '\', imagename],'jpg');
    im_test = imresize(im_test,[240 320]);
    temp = im_test;
    im_test = zeros(240,320,3);
    im_test(:,:,1) = temp;
    im_test(:,:,2) = temp;
    im_test(:,:,3) = temp;
    
    figure(1);image(double(im_test)/255);
    text(5.5,6,['frame: ' num2str(ima)],'color','red');

    x1 = targetdata(i,1);
    y1 = targetdata(i,2);
    x2 = targetdata(i,3);
    y2 = targetdata(i,4);  
    line([x1 x1 x2 x2 x1]', [y1 y2 y2 y1 y1]', 'color', 'w');
    
    if nargin>2
        export_fig([outpath 'result_' num2str(i) '_' foldername]);
    end
    pause(0.1);
    %close all;
end

end

