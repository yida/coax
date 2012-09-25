% Load Image
clc;
close all;

BPdir = 'bright_pixels/';
imagedir = 'images/';
resultdir = 'results/';
file = dir(strcat(imagedir,'*.jpg'));
nfile = size(file,1);

fig = figure;
set(fig,'Pos',[1,500,250,500],'PaperPositionMode','auto');

for fileIdx = 1 : nfile
  tic;
  filename = file(fileIdx).name;
  IMG = imread(strcat(imagedir,filename));
  Intensity = rgb2gray(IMG);

  for spIdx = 5:5:50
    BP = imresize(Intensity,spIdx/100);
    [m n] = size(BP);
    subplot(2,1,1);
    imshow(BP);
    BPC = zeros(1,n);
    [MaxV,MaxIdx]=max(BP,[],1);
    for colIdx = 1 : n
        BPC(1,colIdx) = sum(BP(:,colIdx)==MaxV(1,colIdx));
    end
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
  toc;
end
close all;