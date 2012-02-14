%%
tic;

file = dir(strcat('../images/*.jpg'));

nfile = size(file,1);

%%

for cnt = 1 : nfile
%for cnt = 200;
filename = file(cnt).name;
img = imread(strcat('../images/',filename));
img = rgb2gray(img);

%%

meanIMG = zeros(3,64);
%img = imresize(img,0.1);
[imgN,imgM] = size(img);
% square size

nside = [18,24,36];
for cnt = 1:3
    side = nside(cnt);


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
        %IMG(cnti,cntj) = sum(sum(abs(fliplr(leftpart)-rightpart)));
    end
end

%stdIMG = std(IMG,1);
meanIMG(cnt,(side/2+1):(end-side/2)) = mean(IMG,1);

end

meanIMG(1,:) = meanIMG(1,:)./sum(meanIMG(1,:));
meanIMG(2,:) = meanIMG(2,:)./sum(meanIMG(2,:));
meanIMG(3,:) = meanIMG(3,:)./sum(meanIMG(3,:));

fig = figure(1);
subplot(5,1,1);
imagesc(img);
colormap(gray);
subplot(5,1,2);
plot(1:numel(meanIMG(1,:))-1,meanIMG(1,1:end-1));
%axis([-18/2,numel(meanIMG(1,:))+18/2,min(meanIMG(1,:)),max(meanIMG(1,:))]);
subplot(5,1,3);
plot(1:numel(meanIMG(2,:))-1,meanIMG(2,1:end-1));
%axis([-24/2,numel(meanIMG(2,:))+24/2,min(meanIMG(2,:)),max(meanIMG(2,:))]);
subplot(5,1,4);
plot(1:numel(meanIMG(3,:))-1,meanIMG(3,1:end-1));
%axis([-36/2,numel(meanIMG(3,:))+36/2,min(meanIMG(3,:)),max(meanIMG(3,:))]);
subplot(5,1,5);
plot(1:numel(meanIMG(3,:))-1,0.2*meanIMG(1,1:end-1)+0.5*meanIMG(2,1:end-1)+0.3*meanIMG(3,1:end-1));

% figure;
% surf(IMG);
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
%}
out = 'results/';
print(fig,strcat(out,filename),'-djpeg','-r72');

toc;
end
 %% draw square

