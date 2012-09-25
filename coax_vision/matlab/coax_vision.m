% Load Image
clc;
close all;

imagedir = 'images/';
resultdir = 'results/';
file = dir(strcat(imagedir,'*.jpg'));
nfile = size(file,1);


for fileIdx = 1 : 50 : nfile
  filename = file(fileIdx).name;
  IMG = imread(strcat(imagedir,filename));
  Intensity = rgb2gray(IMG);

  spRange = 20; 
  for spIdx = 5:5:100
    I = imresize(Intensity,spIdx/100);
    [m n] = size(I);
    fig = figure('Pos',[1,500,250,500]);
    set(fig,'PaperPositionMode','auto');
    %set(fig,'PaperPosition',[1,500,250,500]);
    subplot(2,1,1);
    imshow(I);
    BP = I;
    BP(BP~=255)=0;
    BPC = sum(BP,1)./255;
    x = 1 : n;
    xtick = round(linspace(1,n,5));
    subplot(2,1,2);
    plot(x,BPC);
    xlabel('Column');
    ylabel('Bright Pixels');
    BPmax = max(BPC);
    set(gca,'XTick',xtick);
    axis([1 n 0 round(BPmax*1.05)]);
    title(strcat(num2str(n),'x',num2str(m)));
    savefilename = strcat(resultdir,num2str(fileIdx),'r',...
        num2str(spIdx),'P.jpg');
    print(fig,savefilename,'-djpeg','-r72');
%    saveas(fig,savefilename,'jpg');
  end
  imagecat(fileIdx);
  close all;
%{
% Resize Image
H = zeros(10,1);
res = zeros(10,2);
for scaleIdx = 20
    scale = 1.0/scaleIdx;
    img = imresize(I,scale);
    %figure;
    imshow(img);
    [res(scaleIdx/2,1) res(scaleIdx/2,2)] = size(img);
    H(scaleIdx/2) = entropy(img);
end

%%
plot(res(:,1),H);
set(gca,'XTick',flipud(res(:,2)));

%% histograph
hist(img);
%}

end