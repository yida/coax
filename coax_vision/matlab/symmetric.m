%%
tic;

file = dir(strcat('images/*.jpg'));

nfile = size(file,1);

%%
%for i = 1 : nfile
for i = 783;
filename = file(i).name;
img = imread(strcat('images/',filename));
img = rgb2gray(img);
%imshow(img);

%%

% square size
side = 180;

img = imresize(img,0.5);
[imgN,imgM] = size(img);
imshow(img);

Ker = ones(side);
Ker(:,1:side/2) = -1;
% convulation 
IMG = conv2(Ker,double(img));
%image(IMG);
KER = IMG(1+side/2:1+side/2+imgN-side,...
          1+side/2:1+side/2+imgM-side);
%imagesc(KER);      

minKER = min(min(abs(KER)));
[minCol,minRow] = find(KER==minKER|KER==-minKER);
hold on;
for i = 1 : size(minCol,1)
fig = figure(1);
plot([minRow(i),minRow(i),minRow(i)+side,minRow(i)+side,minRow(i)],...
     [minCol(i),minCol(i)+side,minCol(i)+side,minCol(i),minCol(i)]);
end

toc

%out = 'results/';
%print(fig,strcat(out,filename),'-djpeg','-r72');

end
 %% draw square

