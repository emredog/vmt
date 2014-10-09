function [ re ] = test_func3( model, imagename, width, height )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% load('names&sizes');
% i = 12;
% width = sizes(i,2);
% height = sizes(i,1);
% classname = names(i).name;

i = -2 + height/8;
j = -2 + width/8;

if i>7
    a = 1 + ceil( (i-7)/2 );
else
    a = 1;
end

if j>9
    b = 1 + ceil( (j-9)/2 );
else
    b = 1;
end

% loadname = [classname,'model'];
% load(modelname);

%test VOC search
sbin = 8;
interval = 10;
% maxsize = [13 3];%[5 5];
maxsize = [max(a,b) max(a,b)];
padx = max(a,b);%5;
pady = max(a,b);%5;


%Load Filter
% d=load('svmmodel.mat');
% model = d.model;
w = model.SVs' * model.sv_coef;
%w = reshape(w, 14, 6, 32);
%w = reshape(w, 31, 9, 32);
w = reshape(w, i, j, 32);
threshold = model.rho + 0.0;
% filter_size = [14 6];
% filter_size = [31 9];
filter_size = [i j];
max_image_size = [240 320];
clear model;

%Visualize Filter
% pad = 2;
% bs = 20;
% wv = foldHOG(w);
% scale = max(wv(:));
% im = HOGpicture(wv, bs);
% im = imresize(im, 2);
% im = padarray(im, [pad pad], 0);
% im = uint8(im * (255/scale));
%imshow(im);
lambda = 0.1;
alpha = 1;
beta = 0.0001;
%Load Test Image
im_test = imread(imagename,'jpg');
im_test = imresize(im_test,[240 320]);
im_test = double(im_test);
im_test = wlsFilter(im_test, lambda, alpha, beta);%smooth function
pyra = featpyramid_compact(im_test, sbin, interval, maxsize);


%Compute Filter Response
filters = [];
filters{1} = w;
scores = conv_filters(filters, pyra, [1:length(pyra.scales)]);
%pause;

%detect
[det] = scores2det(scores,threshold);
[det_win] = det2win_fortest(det, pyra.scales, sbin, padx, pady, filter_size, max_image_size);
% [picked_detections] = nms(det_win, 0.5);%nms
% plot_boxes = det_win(picked_detections,:);
%plot_boxes = [plot_boxes(:,2) 240-plot_boxes(:,1) plot_boxes(:,4) 240-plot_boxes(:,3)];
if ~isempty(det_win)
    re = 1;
else
    re = 0;
end
% dep = imread(depthname);
% [ plot_boxes ] = otherfilter_func( plot_boxes );
% [plot_boxes] = depthfilter_func2( plot_boxes, dep );
% showboxes(im_test,plot_boxes);
% showboxes2(im_test,plot_boxes,classname);


end

