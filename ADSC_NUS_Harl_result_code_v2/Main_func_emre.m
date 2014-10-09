function [  ] = Main_func_emre(foldername)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
addpath(genpath('All_func'));
load('All_func/HumanDetection_func/thresholds.mat');% load thresholds
% stand_model_ids = [1 2 3];
% tapkeyboards_model_ids = [6 7 8 9];
% dropbag_model_ids = [4 5];
% talkphone_model_ids1 = [20 21 23];
% talkphone_model_ids2 = [22];
% pickup_model_ids = [18 19];
% pickup_model_ids2 = [17];
% shakehand_model_ids = [24];
% shakehand_giveitems_model_ids1 = [25];
% shakehand_giveitems_model_ids2 = [26 27 28];

fprintf('Start detection...\n');
%read images from folder

% video_names = [
% 'vid0018';
% 'vid0033';
% 'vid0048';
% 'vid0057';
% 'vid0072';
% 'vid0087';
% 'vid0102';
% 'vid0117';
% 'vid0125';
% 'vid0131';
% 'vid0144';
% 'vid0159';
% 'vid0174';
% 'vid0009';
% 'vid0021';
% 'vid0036';
% 'vid0051';
% 'vid0060';
% 'vid0075';
% 'vid0090';
% 'vid0105';
% 'vid0120';
% 'vid0126';
% 'vid0132';
% 'vid0147';
% 'vid0162';
% 'vid0177';
% 'vid0012';
% 'vid0024';
% 'vid0039';
% 'vid0052';
% 'vid0063';
% 'vid0078';
% 'vid0093';
% 'vid0108';
% 'vid0122';
% 'vid0127';
% 'vid0135';
% 'vid0150';
% 'vid0165';
% 'vid0180';
% 'vid0015';
% 'vid0027';
% 'vid0042';
% 'vid0053';
% 'vid0066';
% 'vid0081';
% 'vid0096';
% 'vid0111';
% 'vid0123';
% 'vid0129';
% 'vid0138';
% 'vid0153';
% 'vid0168';]


video_names = [
'vid0001';
'vid0002';
'vid0004';
'vid0005';
'vid0007';
'vid0008';
'vid0010';
'vid0011';
'vid0013';
'vid0014';
'vid0016';
'vid0019';
'vid0020';
'vid0022';
'vid0023';
'vid0025';
'vid0026';
'vid0028';
'vid0029';
'vid0031';
'vid0032';
'vid0034';
'vid0035';
'vid0037';
'vid0038';
'vid0040';
'vid0041';
'vid0043';
'vid0044';
'vid0046';
'vid0047';
'vid0049';
'vid0050';
'vid0055';
'vid0056';
'vid0058';
'vid0059';
'vid0061';
'vid0062';
'vid0064';
'vid0065';
'vid0067';
'vid0068';
'vid0070';
'vid0071';
'vid0073';
'vid0074';
'vid0076';
'vid0077';
'vid0079';
'vid0080';
'vid0082';
'vid0083';
'vid0085';
'vid0086';
'vid0088';
'vid0089';
'vid0091';
'vid0092';
'vid0094';
'vid0095';
'vid0097';
'vid0098';
'vid0100';
'vid0101';
'vid0103';
'vid0104';
'vid0106';
'vid0107';
'vid0109';
'vid0110';
'vid0112';
'vid0113';
'vid0115';
'vid0116';
'vid0118';
'vid0119';
'vid0121';
'vid0128';
'vid0133';
'vid0134';
'vid0136';
'vid0137';
'vid0139';
'vid0140';
'vid0142';
'vid0143';
'vid0145';
'vid0146';
'vid0148';
'vid0149';
'vid0151';
'vid0152';
'vid0154';
'vid0155';
'vid0157';
'vid0158';
'vid0160';
'vid0161';
'vid0163';
'vid0164';
'vid0166';
'vid0167';
'vid0169';
'vid0170';
'vid0172';
'vid0173';
'vid0175';
'vid0176';
'vid0178';
'vid0179';
]
% 
% cellData = cellstr(video_names);

%s = regexp(data_path,'\\','split');
%foldername = s{end};

% 
for i=1:length(video_names);
    foldername = video_names(i, 1:7);
    %foldername = 'vid0017';
    data_path = ['/home/emredog/LIRIS-data/training-validation/' foldername];
    humandetection_run(data_path, foldername);
    fprintf('Completed.\n');
end


%data_path2 = ['HumanDetectionResults/' foldername '_result'];

%1. human detection for input folder
%the results saved at HumanDetectionResults


%2. depth mapping for input folder
%the results saved at DepthMappingResults
%---depthmapping_run(data_path, foldername);

%---fprintf('Start analysis...\n');

%3. discussion detection: Action 1
% discusstions = discussiondetect_run( ['HumanDetectionResults\' foldername '_result'], foldername);
%---filter_length = 30;
%---[ standtargets ] = volume2certaintargets( data_path2, filter_length ,stand_model_ids, thresholds(stand_model_ids)+0.8,1);
%---[ discusstions ] = Targets2discussions( standtargets ,foldername,  filter_length );
% showtargets_func(discusstions{1}.data, foldername);
%---if ~isempty(discusstions)
    %---for i = 1:length(discusstions)
        %---discusstions{i} = discusstions{i}.data;
    %---end
%---end



%5. drop bag detection: Action 7
% only base on keypose model
%---filter_length = 5;
%---[ dropbags ] = volume2certaintargets( data_path2, filter_length, dropbag_model_ids, thresholds(dropbag_model_ids)+0.5 );
% if ~isempty(dropbags)
%     for i = 1:length(dropbags)
%         dropbags{i} = dropbags{i}.data;
%     end
% end
%---if ~isempty(dropbags)
%---    deli = [];
%---    for i = 1:length(dropbags)
%---        heights = dropbags{i}.data(:,4) - dropbags{i}.data(:,2);
%---        Ycords = (dropbags{i}.data(:,4) + dropbags{i}.data(:,2))/2;
%---        dropbags{i} = dropbags{i}.data((heights>50)&(Ycords>120),:);
%---        if isempty(dropbags{i})
%---            deli = [deli i];
%---        end
%---    end
%---    dropbags(deli) = [];
%---end
% based on all model and keypose model
% [ dropbags ] = dropbagdetect_run(  data_path, foldername  );

%6. door action detection: Action 4 5 6
% [ dooractions dooractionclasses] = DoorActionDetection_run( data_path, foldername );

%---dooractionclasses = [];
%---dooractions = [];

%---d = dir([data_path '\' '*.jp2']);
%---frames = 1:int16(length(d)/10):length(d);
%---normmap_run( data_path,foldername, frames );
%---sceneclass = sceneclassify_run(foldername, frames);
% if sceneclass == 4
%---[ re ] = sceneclassify_run2( data_path );
% end
%---if (sceneclass ~= 2) || (re ~= 0)
%---    filter_length = 3;
%---    [ standtargets ] = volume2certaintargets( data_path2, filter_length ,stand_model_ids, thresholds(stand_model_ids)+0.8,1);
%---    if length(standtargets)<10
%---        [ standtargets ] = targetslink( standtargets);
%---    end
%---end

%---if re==0 && sceneclass ~=2
%---    [ dooractions, dooractionclasses ] = newdooractiondetection_run(sceneclass, data_path, foldername, standtargets, thresholds );
%---    dooractionclasses = dooractionclasses';
%---else
%---    if re == 1
%---        [  dooractions, dooractionclasses  ] = dooractiondetection_run2 ( foldername, standtargets );
%---        dooractionclasses = dooractionclasses';
%---    else
%---        if re == 2
%---            [  dooractions, dooractionclasses  ] = dooractiondetection_run3 ( standtargets );
%---            dooractionclasses = dooractionclasses';
%         end
%     end
% end
% % sort door actions
% [dooractionclasses, ind] = sort(dooractionclasses);
% dooractions = dooractions(ind);
% 
% %4. tapping keyboard detection: Action 9
% % tapkeyboards = tapkeyboarddetect_run( ['HumanDetectionResults\' foldername '_result'] );
% tapkeyboards = [];
% if sceneclass == 2
%     filter_length = 100;
%     [ tapkeyboards ] = volume2certaintargets( data_path2, filter_length, tapkeyboards_model_ids, thresholds(tapkeyboards_model_ids)+0.0 );
%     if ~isempty(tapkeyboards)
%         for i = 1:length(tapkeyboards)
%             tapkeyboards{i} = tapkeyboards{i}.data;
%         end
%     end
% end
% %7. talks on phone detection: Action 10
% filter_length = 5;
% [ talksphones ] = volume2certaintargets( data_path2, filter_length, talkphone_model_ids1, thresholds(talkphone_model_ids1)+0.65 );
% [ talksphones ] = [ talksphones volume2certaintargets( data_path2, filter_length, ...
%     talkphone_model_ids2, thresholds(talkphone_model_ids2) )];
% if ~isempty(talksphones)
%     for i = 1:length(talksphones)
%         talksphones{i} = talksphones{i}.data;
%     end
% end
% % [ talksphones ] = TalksphoneDetection_run(  ['HumanDetectionResults\' foldername '_result'] );
% 
% %8. pick up item: Action 3
% pickupitems = [];
% pui = 1;
% len = length(dir([data_path '/gray*.jpg']));
% cutlen = 15;
% if ~isempty(talksphones)
%     for i = 1:length(talksphones)
%         if talksphones{i}(1,5) >cutlen
%             pickupitems{pui} = repmat(talksphones{i}(1,:),cutlen,1);
%             for j = 1:cutlen
%                 pickupitems{pui}(j,5) = pickupitems{pui}(j,5) - cutlen -1 + j;
%             end
%             pui = pui + 1;
%         end
%         
%         if talksphones{i}(end,5) < len - cutlen
%             pickupitems{pui} = repmat(talksphones{i}(end,:),cutlen,1);
%             for j = 1:cutlen
%                 pickupitems{pui}(j,5) = pickupitems{pui}(j,5) + j;
%             end
%             pui = pui + 1;
%         end
%     end
% end
% 
% filter_length = 3;
% if sceneclass == 4 
%     %%%standtargets from dooraction part, only in scene~=2!!
%     if re == 1
%         [ keyposes ] = volume2certaintargets( data_path2, filter_length, ...
%             pickup_model_ids(2), thresholds(pickup_model_ids(2))+0.8 );
% %         [ keyposes ] = targetslink( keyposes, 30);
%         temp = [];
%         for j = 1:length(keyposes)
%             temp = [temp;keyposes{j}.data];
%         end
%         keyposes = temp;
%         [ pickupitem] = keyposes_targets2overlap_func( standtargets,keyposes );
%         if ~isempty(pickupitem)
%             for j = 1:length(pickupitem)
%                 pickupitems{pui} = pickupitem{j};
%                 pui = pui + 1;
%             end
%         end
%     else
%         [ pickupitem ] = volume2certaintargets( data_path2, filter_length, ...
%             pickup_model_ids(1), thresholds(pickup_model_ids(1))+0.8 );
%         if ~isempty(pickupitem)
%             for j = 1:length(pickupitem)
%                 pickupitems{pui} = pickupitem{j}.data;
%                 pui = pui + 1;
%             end
%         end    
%     end
% end
% 
% 
% [ re ] = sceneclassify_run2( data_path ,1);
% if re == 3
%     [ pickupitem ] = volume2certaintargets( data_path2, filter_length, ...
%         pickup_model_ids2, thresholds(pickup_model_ids2) );
%     if ~isempty(pickupitem)
%         for j = 1:length(pickupitem)
%             pickupitems{pui} = pickupitem{j}.data;
%             pui = pui + 1;
%         end
%     end
% end
% % 9. Handshake and giveitems: Action 2 8
% HandshakingGiveitems = [];
% Handshakings = [];
% 
% [ HandshakingGiveitems ] = HandshakingGiveitemsDetection_run(  ...
%     ['HumanDetectionResults/' foldername '_result'],thresholds, shakehand_giveitems_model_ids2 );
% 
% [ HandshakingGiveitems2 ] = HandshakingGiveitemsDetection_run(  ...
%     ['HumanDetectionResults/' foldername '_result'],thresholds, shakehand_giveitems_model_ids1,1 );
% 
% HandshakingGiveitems = [HandshakingGiveitems HandshakingGiveitems2];
% 
% [ Handshakings ] = HandshakingGiveitemsDetection_run(  ...
%     ['HumanDetectionResults/' foldername '_result'],thresholds, shakehand_model_ids,1 );
% 
% fprintf('Start output...\n');
% %*. output annos into xml files, except HandshakingGiveitems
% actions = [];
% actionclasses = [];
% 
% % action 1
% actions = [actions discusstions];
% actionclasses = [actionclasses ones(1,length(discusstions))];
% % action 2
% actions = [actions HandshakingGiveitems];
% actionclasses = [actionclasses ones(1,length(HandshakingGiveitems))*2];
% % action 3
% actions = [actions pickupitems];
% actionclasses = [actionclasses ones(1,length(pickupitems))*3];
% % action 4 5 6
% actions = [actions dooractions];
% actionclasses = [actionclasses dooractionclasses'];
% % action 7
% actions = [actions dropbags];
% actionclasses = [actionclasses ones(1,length(dropbags))*7];
% % action 8
% actions = [actions Handshakings];
% actionclasses = [actionclasses ones(1,length(Handshakings))*8];
% actions = [actions HandshakingGiveitems];
% actionclasses = [actionclasses ones(1,length(HandshakingGiveitems))*8];
% % action 9
% actions = [actions tapkeyboards];
% actionclasses = [actionclasses ones(1,length(tapkeyboards))*9];
% % action 10
% actions = [actions talksphones];
% actionclasses = [actionclasses ones(1,length(talksphones))*10];
% 
% if(~isempty(actions))
%     for i=1:length(actions)
%         actions{i} = targetdatainterp( actions{i} );
%         actions{i}(:,1:4) = actions{i}(:,1:4)*2;
%     end
% end

% if(~isempty(actions))
%     videoname_str = ['d1/' foldername];
%     xmlFileName = [foldername,'.xml'];
%     action2xml_func( videoname_str, xmlFileName,actions, actionclasses);
% else
%     videoname_str = ['d1/' foldername];
%     xmlFileName = [foldername,'.xml'];
%     action2xml_func( videoname_str, xmlFileName,[], []);
% end

%end

