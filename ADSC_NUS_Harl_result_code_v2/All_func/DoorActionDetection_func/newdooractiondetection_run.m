function [ dooractions, outputclasses ] = newdooractiondetection_run(sceneclass, data_path, foldername, standtargets, thresholds )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% d = dir([data_path '\' '*.jp2']);
% frames = 1:int16(length(d)/10):length(d);
% % normmap_run( data_path,foldername, frames );
% sceneclass = sceneclassify_run(foldername, frames);
dooractions = [];
outputclasses = [];
model_ids = [];
% scene 1 : D1 D2
% scene 3 : D3 D9
% scene 4 : D6
% scene 5 : D7 D8
if sceneclass~=0
    switch(sceneclass)
        case 1
            model_ids = [10 11];%D1 D2
        case 3
            model_ids = [12 16];%D3 D9
        case 4
            model_ids = [13];%D6
        case 5
            model_ids = [14 15];%D7 D8
    end
%     if length(standtargets)<10
%         [ standtargets ] = targetslink( standtargets);
%     end
    % [ keyposes ] = volume2keyposes( ['HumanDetectionResults\' foldername '_result'], model_ids , thresholds(model_ids)+0.8);
    filter_length = 1;
    data_path2 = ['HumanDetectionResults\' foldername '_result'];
    [ keyposes ] = volume2certaintargets(data_path2 ,filter_length, model_ids , thresholds(model_ids)+0.8);
    
    if ~isempty(keyposes) && (sceneclass == 3 || sceneclass == 5)
        deli = [];
        for i = 1:length(keyposes)
            heights = keyposes{i}.data(:,4) - keyposes{i}.data(:,2);
            Ycords = (keyposes{i}.data(:,4) + keyposes{i}.data(:,2))/2;
            keyposes{i}.data = keyposes{i}.data((heights>125)&(Ycords>80)&(Ycords<180)&(size(keyposes{i}.data,1)>=2),:);
            if isempty(keyposes{i}.data)
                deli = [deli i];
            end
        end
        keyposes(deli) = [];
    end
%     [ keyposes ] = targetsinterp( keyposes );
    
    [ keyposes ] = targetslink( keyposes, 30);
    temp = [];
    for i = 1:length(keyposes)
        temp = [temp;keyposes{i}.data];
    end
    keyposes = temp;
    
    if sceneclass==3 || sceneclass==5
        ifscene35 = 1;
    else
        ifscene35 = 0;
    end
    
    [ dooractions, outputclasses ] = newdooractiondetection_func( standtargets,keyposes,ifscene35, foldername );

    if sceneclass == 4
        for i = 1:length(outputclasses)
            outputclasses(i) = 4;
        end
    end
    
end

end

