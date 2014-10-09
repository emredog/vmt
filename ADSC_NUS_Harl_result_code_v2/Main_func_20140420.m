function [  ] = Main_func_20140420( data_path )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
addpath(genpath('All_func'));
load('All_func/HumanDetection_func/thresholds.mat');% load thresholds
stand_model_ids = [1 2 3];
tapkeyboards_model_ids = [6 7 8 9];
dropbag_model_ids = [4 5];
talkphone_model_ids1 = [20 21 23];
talkphone_model_ids2 = [22];
pickup_model_ids = [18 19];
pickup_model_ids2 = [17];
shakehand_model_ids = [24];
shakehand_giveitems_model_ids1 = [25];
shakehand_giveitems_model_ids2 = [26 27 28];

fprintf('Start detection...\n');
%read images from folder

s = regexp(data_path,'/','split');
foldername = s{end};
% data_path = ['C:\Users\Pei.Yong\Downloads\wget-1.10.2b\d1-test\test\' foldername];
data_path2 = ['HumanDetectionResults/' foldername '_result'];
trackletDir=['Tracklets/test_20140425/' foldername '/']; 
if ~exist(trackletDir,'dir')
    mkdir(trackletDir);
end

%1. human detection for input folder
%the results saved at HumanDetectionResults
% humandetection_run(data_path, foldername);

%2. depth mapping for input folder
%the results saved at DepthMappingResults
%depthmapping_run(data_path, foldername);

fprintf('Start analysis...\n');

%3. discussion detection: Action 1
% discusstions = discussiondetect_run( ['HumanDetectionResults\' foldername '_result'], foldername);
filter_length = 30;
[ standtargets ] = volume2certaintargets( data_path2, filter_length ,stand_model_ids, thresholds(stand_model_ids)+0.8,1);

[ discusstions ] = Targets2discussions( standtargets ,foldername,  filter_length );

% % % % SAVE TRACKLETS HERE % % % %
for i=1:length(discusstions)
    filename = [trackletDir 'discussions-' num2str(i) '.csv'];
    csvwrite(filename, discusstions{i}.data);
end
%if discussions is empty, try to write standtargets:
if isempty(discusstions)
    for i=1:length(standtargets)
        filename = [trackletDir 'standtargets-' num2str(i) '.csv'];
        csvwrite(filename, standtargets{i}.data);
    end
end


% showtargets_func(discusstions{1}.data, foldername);
%if ~isempty(discusstions)
%    for i = 1:length(discusstions)
%        discusstions{i} = discusstions{i}.data;
%    end
%end



%5. drop bag detection: Action 7
% only base on keypose model
filter_length = 5;
[ dropbags ] = volume2certaintargets( data_path2, filter_length, dropbag_model_ids, thresholds(dropbag_model_ids)+0.5 );


if ~isempty(dropbags)
    deli = [];
    for i = 1:length(dropbags)
        heights = dropbags{i}.data(:,4) - dropbags{i}.data(:,2);
        Ycords = (dropbags{i}.data(:,4) + dropbags{i}.data(:,2))/2;
        dropbags{i} = dropbags{i}.data((heights>50)&(Ycords>120),:);
        if isempty(dropbags{i})
            deli = [deli i];
        end
    end
    dropbags(deli) = [];
end

% % % % SAVE TRACKLETS HERE % % % %
for i=1:length(dropbags)
    filename = [trackletDir 'dropbags-' num2str(i) '.csv'];
    csvwrite(filename, dropbags{i});
end

% if ~isempty(dropbags)
%     for i = 1:length(dropbags)
%         dropbags{i} = dropbags{i}.data;
%     end
% end


% based on all model and keypose model
%[ dropbags ] = dropbagdetect_run(  data_path, foldername  );

%6. door action detection: Action 4 5 6
% [ dooractions dooractionclasses] = DoorActionDetection_run( data_path, foldername );

dooractionclasses = [];
dooractions = [];

d = dir([data_path '/' '*.jp2']);
frames = 1:int16(length(d)/10):length(d);

%normmap_run( data_path,foldername, frames ); % already done it
sceneclass = sceneclassify_run(foldername, frames); 

% if sceneclass == 4
[ re ] = sceneclassify_run2( data_path );
% end
if (sceneclass ~= 2) || (re ~= 0)
    filter_length = 3;
    [ standtargets ] = volume2certaintargets( data_path2, filter_length ,stand_model_ids, thresholds(stand_model_ids)+0.8,1);
    if length(standtargets)<10
        [ standtargets ] = targetslink( standtargets);
    end
    
    for i=1:length(standtargets)
        
        filename = [trackletDir 'standtargets_2-' num2str(i) '.csv'];
        csvwrite(filename, standtargets{i}.data);
    end
end


%-------------------------------------------------------------------------------
% -------- WE DONT NEED THIS PART ----------------------------------------------

%if re==0 && sceneclass ~=2
%    [ dooractions, dooractionclasses ] = newdooractiondetection_run(sceneclass, data_path, foldername, standtargets, thresholds ); %zaman aldi
%    dooractionclasses = dooractionclasses';
%else
%    if re == 1
%        [  dooractions, dooractionclasses  ] = dooractiondetection_run2 ( foldername, standtargets );
%        dooractionclasses = dooractionclasses';
%    else
%        if re == 2
%            [  dooractions, dooractionclasses  ] = dooractiondetection_run3 ( standtargets );
%            dooractionclasses = dooractionclasses';
%        end
%    end
%end



% sort door actions
% [dooractionclasses, ind] = sort(dooractionclasses);
% dooractions = dooractions(ind);

%-------------------------------------------------------------------------------

%4. tapping keyboard detection: Action 9
% tapkeyboards = tapkeyboarddetect_run( ['HumanDetectionResults\' foldername '_result'] );
tapkeyboards = [];
if sceneclass == 2
    filter_length = 100;
    [ tapkeyboards ] = volume2certaintargets( data_path2, filter_length, tapkeyboards_model_ids, thresholds(tapkeyboards_model_ids)+0.0 );
    
    for i=1:length(tapkeyboards)
        filename = [trackletDir 'tapkeyboards-' num2str(i) '.csv'];
        csvwrite(filename, tapkeyboards{i}.data);
    end
    
    if ~isempty(tapkeyboards)
        for i = 1:length(tapkeyboards)
            tapkeyboards{i} = tapkeyboards{i}.data;
        end
    end
    
    
end
%7. talks on phone detection: Action 10
filter_length = 5;
[ talksphones ] = volume2certaintargets( data_path2, filter_length, talkphone_model_ids1, thresholds(talkphone_model_ids1)+0.65 );



[ talksphones ] = [ talksphones volume2certaintargets( data_path2, filter_length, ...
    talkphone_model_ids2, thresholds(talkphone_model_ids2) )];

% % % % SAVE TRACKLETS HERE % % % %
for i=1:length(talksphones)
        filename = [trackletDir 'talksphones-' num2str(i) '.csv'];
        csvwrite(filename, talksphones{i}.data);
end


if ~isempty(talksphones)
    for i = 1:length(talksphones)
        talksphones{i} = talksphones{i}.data;
    end
end

% [ talksphones ] = TalksphoneDetection_run(  ['HumanDetectionResults\' foldername '_result'] );

%8. pick up item: Action 3
pickupitems = [];
pui = 1;
len = length(dir([data_path '/gray*.jpg']));
cutlen = 15;
if ~isempty(talksphones)
    for i = 1:length(talksphones)
        if talksphones{i}(1,5) >cutlen
            pickupitems{pui} = repmat(talksphones{i}(1,:),cutlen,1);
            for j = 1:cutlen
                pickupitems{pui}(j,5) = pickupitems{pui}(j,5) - cutlen -1 + j;
            end
            pui = pui + 1;
        end
        
        if talksphones{i}(end,5) < len - cutlen
            pickupitems{pui} = repmat(talksphones{i}(end,:),cutlen,1);
            for j = 1:cutlen
                pickupitems{pui}(j,5) = pickupitems{pui}(j,5) + j;
            end
            pui = pui + 1;
        end
    end
end

% % % % SAVE TRACKLETS HERE % % % %
for i=1:length(pickupitems)
    filename = [trackletDir 'pickupitem_1-' num2str(i) '.csv'];
    csvwrite(filename, pickupitems{i});
end

filter_length = 3;
if sceneclass == 4 
    %%%standtargets from dooraction part, only in scene~=2!!
    if re == 1
        [ keyposes ] = volume2certaintargets( data_path2, filter_length, ...
            pickup_model_ids(2), thresholds(pickup_model_ids(2))+0.8 );
        
        for i=1:length(keyposes)
            filename = [trackletDir 'keyposes-' num2str(i) '.csv'];
            csvwrite(filename, keyposes{i}.data);
        end
        
%         [ keyposes ] = targetslink( keyposes, 30);
        temp = [];
        for j = 1:length(keyposes)
            temp = [temp;keyposes{j}.data];
        end
        keyposes = temp;
        [ pickupitem] = keyposes_targets2overlap_func( standtargets,keyposes );
        if ~isempty(pickupitem)
            for j = 1:length(pickupitem)
                pickupitems{pui} = pickupitem{j};
                pui = pui + 1;
            end
        end
    else
        [ pickupitem ] = volume2certaintargets( data_path2, filter_length, ...
            pickup_model_ids(1), thresholds(pickup_model_ids(1))+0.8 );                
        
        if ~isempty(pickupitem)
            for j = 1:length(pickupitem)
                pickupitems{pui} = pickupitem{j}.data;
                pui = pui + 1;
            end
            
            % % % % SAVE TRACKLETS HERE % % % %
            for i=1:length(pickupitems)
                filename = [trackletDir 'pickupitem_2-' num2str(i) '.csv'];
                csvwrite(filename, pickupitems{i});
            end
        end    
    end
end


[ re ] = sceneclassify_run2( data_path ,1);
if re == 3
    [ pickupitem ] = volume2certaintargets( data_path2, filter_length, ...
        pickup_model_ids2, thresholds(pickup_model_ids2) );
    
    % % % % SAVE TRACKLETS HERE % % % %
    for i=1:length(pickupitem)
            filename = [trackletDir 'pickupitem_3-' num2str(i) '.csv'];
            csvwrite(filename, pickupitem{i}.data);
    end
    
    if ~isempty(pickupitem)
        for j = 1:length(pickupitem)
            pickupitems{pui} = pickupitem{j}.data;
            pui = pui + 1;
        end
    end
end



% 9. Handshake and giveitems: Action 2 8
HandshakingGiveitems = [];
Handshakings = [];

[ HandshakingGiveitems ] = HandshakingGiveitemsDetection_run(  ...
    ['HumanDetectionResults/' foldername '_result'],thresholds, shakehand_giveitems_model_ids2 );

[ HandshakingGiveitems2 ] = HandshakingGiveitemsDetection_run(  ...
    ['HumanDetectionResults/' foldername '_result'],thresholds, shakehand_giveitems_model_ids1,1 );

HandshakingGiveitems = [HandshakingGiveitems HandshakingGiveitems2];

% % % % SAVE TRACKLETS HERE % % % %
for i=1:length(HandshakingGiveitems)
    filename = [trackletDir 'HandshakeGiveItems-' num2str(i) '.csv'];
    csvwrite(filename, HandshakingGiveitems{i});
end

[ Handshakings ] = HandshakingGiveitemsDetection_run(  ...
    ['HumanDetectionResults/' foldername '_result'],thresholds, shakehand_model_ids,1 );

% % % % SAVE TRACKLETS HERE % % % %
for i=1:length(Handshakings)
    filename = [trackletDir 'Handshakings-' num2str(i) '.csv'];
    csvwrite(filename, Handshakings{i});
end


fprintf([foldername ': Tracklet extracted.\n']);
return;
%-----------------------------------------------------
% EXECUTION STOPS HERE -------------------------------
%-----------------------------------------------------



fprintf('Start output...\n');
%*. output annos into xml files, except HandshakingGiveitems
actions = [];
actionclasses = [];

% action 1
actions = [actions discusstions];
actionclasses = [actionclasses ones(1,length(discusstions))];
% action 2
actions = [actions HandshakingGiveitems];
actionclasses = [actionclasses ones(1,length(HandshakingGiveitems))*2];
% action 3
actions = [actions pickupitems];
actionclasses = [actionclasses ones(1,length(pickupitems))*3];
% action 4 5 6
actions = [actions dooractions];
actionclasses = [actionclasses dooractionclasses'];
% action 7
actions = [actions dropbags];
actionclasses = [actionclasses ones(1,length(dropbags))*7];
% action 8
actions = [actions Handshakings];
actionclasses = [actionclasses ones(1,length(Handshakings))*8];
actions = [actions HandshakingGiveitems];
actionclasses = [actionclasses ones(1,length(HandshakingGiveitems))*8];
% action 9
actions = [actions tapkeyboards];
actionclasses = [actionclasses ones(1,length(tapkeyboards))*9];
% action 10
actions = [actions talksphones];
actionclasses = [actionclasses ones(1,length(talksphones))*10];

if(~isempty(actions))
    for i=1:length(actions)
        actions{i} = targetdatainterp( actions{i} );
        actions{i}(:,1:4) = actions{i}(:,1:4)*2;
    end
end

if(~isempty(actions))
    videoname_str = ['d1/' foldername];
    xmlFileName = [foldername,'.xml'];
    action2xml_func( videoname_str, xmlFileName,actions, actionclasses);
else
    videoname_str = ['d1/' foldername];
    xmlFileName = [foldername,'.xml'];
    action2xml_func( videoname_str, xmlFileName,[], []);
end

end

