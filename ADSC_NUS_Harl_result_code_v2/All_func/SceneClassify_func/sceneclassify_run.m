function [ sceneclass ] = sceneclassify_run( imagefolder ,frames)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
data_path=['NormMapResults/' imagefolder];
load sceneclassify_model;

resdir='SceneclassifyResults/'; 
if ~exist(resdir,'dir')
    mkdir(resdir);
end

outputpath = [resdir imagefolder];
if ~exist(outputpath,'dir')
    mkdir(outputpath);
end

% normlist = dir([data_path,'\',imagefolder,'_*','jpg']);

%extract features
for i = 1:length(frames)
    imagename = [data_path '/' sprintf('norm-%06d.jpg',frames(i))];
    I = imread(imagename);
    features(i,:) = ExtractNFeatures(I);
end

conditions = features(:,16:22);
features = features(:,1:15);

%[predicted_label, accuracy, decision_valuesprob_estimates] = svmpredict(zeros(size(features,1),1), features, model);
%save([outputpath,'/','Sceneclassify_Result_',imagefolder,'.mat'], 'predicted_label');

load([outputpath,'/','Sceneclassify_Result_',imagefolder,'.mat'])

sceneclass = 0;
low_tho = 5;
if sum(predicted_label ==1)>=low_tho
    sceneclass = 1;
else
    if sum(predicted_label ==3)>=low_tho
        sceneclass = 3;
    else
        if sum(predicted_label ==5)>=low_tho
            sceneclass = 5;
        else
            if sum(predicted_label ==2)>=low_tho
                sceneclass = 2;
            else
                if sum(predicted_label ==4)>=low_tho
                    sceneclass = 4;
                end
            end
        end
    end
end

if sceneclass == 2
    temp = [];
    for i = 1:length(frames)
        %for scene 3
        if (conditions(i,3) > 0.8 && conditions(i,4) > 0.5 ) ||  ...
                (conditions(i,5) > 0.65 && conditions(i,5) > 0.7)
            temp = [temp 3];
        end
        
        %for scene 5  
        if conditions(i,2) > 0.8 && conditions(i,1) > 0.5 
            temp = [temp 5];
        end
    end
    if sum(temp == 3) >4 & sum(temp == 5) <5
        sceneclass = 3;
    end
    if sum(temp == 5) >4 & sum(temp == 3) <5
        sceneclass = 5;
    end
end

if sceneclass ==4
    temp = [];
    for i = 1:length(frames)    
        %for scene 5  
        if conditions(i,6) > 0.8 && conditions(i,7) > 0.8 
            temp = [temp 5];
        end
    end
    if sum(temp == 5) >7
        sceneclass = 5;
    end
end

end

