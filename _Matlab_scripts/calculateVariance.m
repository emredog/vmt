function variances = calculateVariance

numFiles = 41;

bigMat = zeros(numFiles, 480, 640);


%     inline float raw_depth_to_meters(int raw_depth) const
% 	{
%         if (raw_depth < 2047)
% 			return 1.0 / (raw_depth * -0.0030711016 + 3.3309495161);
% 		return 0;
% 	}

for k=1:numFiles
    fileName = sprintf('/home/emredog/LIRIS-data/training-validation/vid0028/depth-%06d.jp2', k);
    A = imread(fileName);
    
    A = bitand(A, 65504); %zero out last 5 bits
    A = A / 32; %bitshift by 5 bits
    LOG = A < 2046; % zero out 2046 and 2047 values    
    A = uint16(1000.0 ./ (double(A) * -0.0030711016 + 3.3309495161));
    A = A .* uint16(LOG);
    
    bigMat(k,:,:) = A;
end

variances = zeros(480, 640);

for i=1:480
    for j=1:640        
        vec = bigMat(:, i, j);
        vec = vec(vec~=0); % remove zero elements of vec
        variances(i, j) = var(vec);
    end
end

return;