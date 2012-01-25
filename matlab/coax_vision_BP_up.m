% Load Image
clc;
close all;

if isdir('bright_pixels/up') == 0
  !mkdir -p bright_pixels/up
end

BPdir = 'bright_pixels/up/';
imagedir = 'images/';
resultdir = 'results/';
file = dir(strcat(imagedir,'*.jpg'));
nfile = size(file,1);


for fileIdx = 1 : nfile
  filename = file(fileIdx).name
  IMG = imread(strcat(imagedir,filename));
  Intensity = rgb2gray(IMG);
  [Im,In] = size(Intensity);
  % up half image
  Intensity = Intensity(1:Im/2,:);

  spRange = 20; 
  for spIdx = 5:5:50
    I = imresize(Intensity,spIdx/100);
    [m n] = size(I);
    fig = figure('Visible','off','Pos',[1,500,250,500]);
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
    BPmin = min(BPC);
    set(gca,'XTick',xtick);
    axis([1 n BPmin*0.95 BPmax*1.05]);
    title(strcat(num2str(n),'x',num2str(m)));
    savefilename = strcat(resultdir,num2str(fileIdx),'r',...
        num2str(spIdx),'P.jpg');
    print(fig,savefilename,'-djpeg','-r72');
  end
  imagecat(fileIdx,BPdir);
  close all;
end
