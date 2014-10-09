matFiles = ReadFileNames('/home/emredog/Documents/ADSC_NUS_Harl_result_code_v2/HumanDetectionResults/training-validation/');

for i=1:length(matFiles)
    matPath = matFiles{i};
    load(matPath);
    fid = fopen([matPath '.txt'], 'w+');
    
    for r=1:28
        fprintf(fid, '<%d>\n', r);
        plot_box =  plot_boxes{1,r};
        [m, n] = size(plot_box);
        if m > 0
            for p=1:m
                fprintf(fid, '%d:', p);
                for c=1:n
                    fprintf(fid, '%.4f;', plot_box(p, c));
                end
                fprintf(fid, '\n');
            end
        end
        fprintf(fid, '</%d>\n', r);
    end
    fclose(fid);
    fprintf('.');
end