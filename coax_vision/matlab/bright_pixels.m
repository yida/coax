function [BP] = bright_pixels(img,slice_width,method)

%
% Function bright_pixels compute mean or median of fixed width slices 
% method : 'median','mean'
% slice_width : width of shifting slice 
%

%{
% Default method mean if not named
if (nargin < 3)
    method = 'mean';
end

% Default slice width 5 if not named
if (nargin < 2)
    slice_width = 1;
end
%}

%slice summation

[~,m] = size(img);
shift = slice_width - 1;
BP = zeros(1,m);
for colidx = 1 : m
    startpnt = max(colidx-shift,1); 
    endpnt = min(colidx+shift,m);
    slice = img(:,startpnt:endpnt);
    if (strcmp(method,'mean')==1)
        BP(1,colidx) = mean(mean(slice));
    elseif (strcmp(method,'median')==1)
        BP(1,colidx) = median(median(slice));
    end
end