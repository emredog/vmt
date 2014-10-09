function [ HandshakingGiveitems ] = HandshakingGiveitemsDetection_run(  data_path2 , thresholds, keypose_model_ids, ifhighscorerule)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if nargin< 4
    ifhighscorerule = 0;
end

keytho = 3.0;

filter_length = 5;
[ targets ] = volume2certaintargets( data_path2, filter_length ,[1 2 3], thresholds([1 2 3])+0.8,1);
% keypose_model_ids = [24 25 26 27 28];

if ifhighscorerule == 0
    [ keyposes ] = volume2keyposes( data_path2, keypose_model_ids );%%don't know the id yet
    [ HandshakingGiveitems ] = HGDetection_func( targets ,keyposes);
else
    [ keyposes ] = volume2keyposes( data_path2, keypose_model_ids,thresholds(keypose_model_ids), keytho);%%don't know the id yet
    [ HandshakingGiveitems ] = HGDetection_func( targets ,keyposes);
%     [ keyposes ] = volume2keyposes( data_path2, keypose_model_ids);%%don't know the id yet
    [ HandshakingGiveitems2 ] = volume2certaintargets( data_path2, 1 ,keypose_model_ids, keytho*ones(1,length(keypose_model_ids)) );
    
    if ~isempty(HandshakingGiveitems2)
        for i = 1:length(HandshakingGiveitems2)
            HandshakingGiveitems2{i} = HandshakingGiveitems2{i}.data;
            widths = HandshakingGiveitems2{i}(:,3) - HandshakingGiveitems2{i}(:,1);
            heights = HandshakingGiveitems2{i}(:,4) - HandshakingGiveitems2{i}(:,2);
            HandshakingGiveitems2{i}(:,1) = max(HandshakingGiveitems2{i}(:,1) - widths,1);
            HandshakingGiveitems2{i}(:,3) = min(HandshakingGiveitems2{i}(:,3) + widths,320);
            HandshakingGiveitems2{i}(:,2) = max(HandshakingGiveitems2{i}(:,2) - heights,1);
            HandshakingGiveitems2{i}(:,4) = min(HandshakingGiveitems2{i}(:,4) + heights,240);
        end
    end
    
    HandshakingGiveitems = [HandshakingGiveitems HandshakingGiveitems2];
end

end
