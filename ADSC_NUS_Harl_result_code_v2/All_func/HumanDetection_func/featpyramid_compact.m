function pyra = featpyramid_compact(im, sbin, interval, maxsize)

% pyra = featpyramid(im, model, padx, pady);
% Compute feature pyramid.
%
% pyra.feat{i} is the i-th level of the feature pyramid.
% pyra.scales{i} is the scaling factor used for the i-th level.
% pyra.feat{i+interval} is computed at exactly half the resolution of feat{i}.
% first octave halucinates higher resolution data.
% padx,pady optionally pads each level of the feature pyramid


padx = ceil(maxsize(2));
pady = ceil(maxsize(1));

sc = 2 ^(1/interval);
imsize = [size(im, 1) size(im, 2)];
max_scale = 1 + floor(log(min(imsize)/(5*sbin))/log(sc));
pyra.feat = cell(max_scale + interval, 1);
pyra.scales = zeros(max_scale + interval, 1);
pyra.imsize = imsize;

% our resize function wants floating point values
im = double(im);
for i = 1:interval
  scaled = imresize(im, 1/sc^(i-1));
  % "first" 2x interval
  pyra.feat{i} = features2(scaled, sbin/2);%disp(size(pyra.feat{i}));
  pyra.scales(i) = 2/sc^(i-1);
  % "second" 2x interval
  pyra.feat{i+interval} = features2(scaled, sbin);
  pyra.scales(i+interval) = 1/sc^(i-1);
  % remaining interals
  for j = i+interval:interval:max_scale
    scaled = imresize(scaled, 0.5);
    pyra.feat{j+interval} = features2(scaled, sbin);
    pyra.scales(j+interval) = 0.5 * pyra.scales(j);
  end
end

for i = 1:length(pyra.feat)
  % add 1 to padding because feature generation deletes a 1-cell
  % wide border around the feature map
  pyra.feat{i} = padarray(pyra.feat{i}, [pady+1 padx+1 0], 0);
  % write boundary occlusion feature
  pyra.feat{i}(1:pady+1, :, 14) = 1;
  pyra.feat{i}(end-pady:end, :, 14) = 1;
  pyra.feat{i}(:, 1:padx+1, 14) = 1;
  pyra.feat{i}(:, end-padx:end, 14) = 1;
end
pyra.padx = padx;
pyra.pady = pady;
