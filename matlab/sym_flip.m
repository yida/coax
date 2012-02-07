%%
%close all;
file = dir(strcat('image/fly2/*.jpg'));

nfile = size(file,1);

%%
%for i = 1 : nfile
for i = 200
filename = file(i).name;
img = imread(strcat('image/fly2/',filename));
img = rgb2gray(img);
%imshow(img);

%%

% square size
side = 90;

img = imresize(img,0.25);

img = img(:,21:140);
[imgN,imgM] = size(img);
%img(img>180) = 50;
%imshow(img);

lrpeak = zeros(1,imgM);
udpeak = zeros(1,imgM);
stdpx = zeros(1,imgM);
meanpx = zeros(1,imgM);
diffpx = zeros(1,imgM);
for ax = 2 : imgM-1
    leftLen = ax - 1;
    rightLen = imgM - ax;
    Len = min(leftLen,rightLen);
    leftpart = img(:,ax-Len:ax-1);
    rightpart = img(:,ax+1:ax+Len);   
    diffpx(ax) = abs(sum(sum(fliplr(leftpart)))-sum(sum(rightpart)));
    %diffpx(ax) = abs(sum(sum(abs((leftpart-rightpart)))));
    stdpx(ax) = min(std(std(double(img(:,ax-min(10,Len):ax)))),std(std(double(img(:,ax:ax+min(10,Len))))));
    meanpx(ax) = min(mean(mean(double(img(:,ax-min(5,Len):ax)))),mean(mean(double(img(:,ax:ax+min(5,Len))))));
    %stdpx(ax) = min(std(std(double(leftpart))),std(std(double(rightpart))));
    %uppart = img(1:imgN/2,ax-min(Len,5):ax+min(Len,5));
    %lowerpart = img(imgN/2+1:end,ax-min(Len,5):ax+min(Len,5));
    R = corrcoef(double(fliplr(leftpart)),double(rightpart));
    %lrpeak(ax) = sum(sum((fliplr(leftpart)-rightpart).^2));
    lrpeak(ax) = R(1,2);
    %R = corrcoef(double(flipud(uppart)),double(lowerpart));
    %udpeak(ax) = R(1,2);
end
stdpx = stdpx./max(stdpx);
%stdpx(stdpx>0.2)=0.2;
%%
bench = log(stdpx) + 5*lrpeak;
[C,I] = max(bench);

%{
fig = figure(1);
imagesc(img);
colormap(gray);
hold on;
plot([I,I],[1,imgN],'LineWidth',2);
%}
%hold off;


figure;
subplot(2,1,1);
imagesc(img);
colormap(gray);
subplot(2,1,2);
%plot(1:imgM,abs(udpeak));
%subplot(4,1,3);
%plot(1:imgM,lrpeak);
%subplot(4,1,4);
%plot(1:imgM,lrpeak-abs(udpeak));
%subplot(4,1,3);
plot(1:imgM,diffpx);
% subplot(4,1,4);
% plot(1:imgM,bench);

%toc

%out = 'results3/';
%print(fig,strcat(out,filename),'-djpeg','-r72');



%%

% %subplot(3,1,1);
% figure;
% imagesc(img);
% colormap(gray);
% hold on;
% %hold on;
% line = zeros(2,120);
% nline = 1;
% for cnt = 2:119
%  if (lrpeak(cnt-1) <= lrpeak(cnt)) && (lrpeak(cnt) >= lrpeak(cnt+1))
%      plot([cnt,cnt],[1,120],'LineWidth',2);
%      line(:,nline) = [cnt,lrpeak(cnt)];
%      nline = nline + 1;
%  end
% end
% 
% 
% line = line(:,1:nline-1);
%line = line(:,2:end-1);
%{
[C,I] = max(line(2,:));
I = line(1,I);
%plot([I,I],[1,120],'LineWidth',2);
MAX=1;
MAXval = 0;
MAXdiff = 0;
for cnt = 2:size(line,2)-1
    if ((line(2,cnt)>=line(2,cnt-1)) && (line(2,cnt)>=line(2,cnt+1))) || ((line(2,cnt)<=line(2,cnt-1)) && (line(2,cnt)<=line(2,cnt+1)))
        diff = abs(line(2,cnt) - line(2,cnt-1))+abs(line(2,cnt)-line(2,cnt-1));
        if (line(2,cnt)>MAXval)
            %disp(cnt);
            MAXval = line(2,cnt);
            MAX = cnt;
            MAXdiff = diff;
 %{
        elseif (diff>MAXdiff)
            MAXval = line(2,cnt);
            MAX = cnt;
            MAXdiff = diff;
%}
        end
        
    end
end
plot([line(1,MAX),line(1,MAX)],[1,120],'LineWidth',2);
%out = 'results4/';
%print(fig,strcat(out,filename),'-djpeg','-r72');
%}
end

 %% draw square

