function variances = calculateVariance

numFiles = 41;

bigMat = zeros(numFiles, 480, 640);

for k=1:numFiles
    fileName = sprintf('/home/emredog/LIRIS-data/training-validation/vid0028/depth-%06d.jp2', k);
    A = imread(fileName);
    
    A = bitand(A, 65504);
    A = A / 32;
    
    bigMat(k,:,:) = A;
end

variances = zeros(480, 640);

for i=1:480
    for j=1:640
        vec = bigMat(:, i, j);
        variances(i, j) = var(vec);
    end
end

return;