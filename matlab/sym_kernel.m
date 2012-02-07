%%
tic;

file = dir(strcat('image/fly2/*.jpg'));

nfile = size(file,1);

%%
%for i = 1 : nfile
for cnt = 300;
filename = file(cnt).name;
img = imread(strcat('image/fly2/',filename));
img = rgb2gray(img);

%%

% square size
side = 150;

img = imresize(img,0.5);
[imgN,imgM] = size(img);
%imshow(img);

Ker = ones(side);
Ker(:,1:side/2) = -1;
% convulation 
%IMG = conv2(Ker,double(img));
IMG = zeros(imgN-side,imgM-side);
for cnti = 1 : (imgN - side - 1)
    for cntj = 1 : (imgM - side -1)
        ker = img(cnti:cnti+side-1,cntj:cntj+side-1);
        leftpart = ker(:,1:end/2);
        rightpart = ker(:,end/2+1:end);
        %R = corrcoef(double(fliplr(leftpart)),double(rightpart));
        %IMG(cnti,cntj) = R(1,2);
        IMG(cnti,cntj) = abs(sum(sum(fliplr(leftpart)))-sum(sum(rightpart)));
    end
end

meanIMG = mean(IMG,1);
figure;
subplot(2,1,1);
imagesc(img);
subplot(2,1,2);
plot(1:numel(meanIMG)-1,meanIMG(1:end-1));
%figure;
%image(IMG);
%KER = IMG(1+side/2:1+side/2+imgN-side,...
%          1+side/2:1+side/2+imgM-side);
%imagesc(KER);      

%{
[minCol,minRow] = min(min(abs(IMG)));
%[minCol,minRow] = find(KER==minKER|KER==-minKER);
fig =figure;
imagesc(img);
colormap(gray);
hold on;
for i = 1 : size(minCol,1)
%fig = figure(1);
plot([minRow(i),minRow(i),minRow(i)+side,minRow(i)+side,minRow(i)],...
     [minCol(i),minCol(i)+side,minCol(i)+side,minCol(i),minCol(i)]);
end

toc

out = 'results1/';
print(fig,strcat(out,filename),'-djpeg','-r72');
%}
toc;
end
 %% draw square

