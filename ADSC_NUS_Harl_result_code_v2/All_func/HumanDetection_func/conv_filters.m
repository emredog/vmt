%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% compute all filter responses (filter score pyramids)
function scores = conv_filters(filters, pyra, levels)
% model    object model
% pyra     feature pyramid
% latent   true => latent positive detection mode
% bbox     ground truth bbox
% overlap  overlap threshold

scores = [];

for level = levels
  % compute filter response for all filters at this level
  r = fconv(pyra.feat{level}, filters, 1, length(filters));
  % find max response array size for this level
  s = [-inf -inf];
  for i = 1:length(r)
    s = max([s; size(r{i})]);
  end
  % set filter response as the score for each filter terminal
  for i = 1:length(r)
    % normalize response array size so all responses at this 
    % level have the same dimension
    spady = s(1) - size(r{i},1);
    spadx = s(2) - size(r{i},2);
    r{i} = padarray(r{i}, [spady spadx], -inf, 'post');
  end

  scores{level} = r;
end