% Load Image
clc;
close all;
clear all;

if isdir('entropy/up') == 0
  !mkdir -p entropy/up
end

Endir = 'entropy/up/';
imagedir = 'images/';
resultdir = 'results/';
file = dir(strcat(imagedir,'*.jpg'));
nfile = size(file,1);


for fileIdx = 1 : nfile
%for fileIdx = 1436
  filename = file(fileIdx).name;
  IMG = imread(strcat(imagedir,filename));
  Intensity = rgb2gray(IMG);
  [Im,In] = size(Intensity);
  Intensity = Intensity(1:Im/2,:);

  spRange = 20; 
  for spIdx = 5:5:50
  %for spIdx = 100
    I = imresize(Intensity,spIdx/100);
    [m n] = size(I);
    fig = figure('Visible','off','Pos',[1,500,250,500]);
    set(fig,'PaperPositionMode','auto');
    %set(fig,'PaperPosition',[1,500,250,500]);
    subplot(2,1,1);
    imshow(I);
    Entropy = zeros(n,1);
    pshift = 10;
    for col=1:n
        leftpnt = max(1,col-pshift);
        rightpnt = min(n,col+pshift);
        slice = I(:,leftpnt:rightpnt);
        Entropy(col,1) = entropy(slice);
    end
    x = 1 : n;
    subplot(2,1,2);
    xtick = round(linspace(1,n,5));
    plot(x,Entropy);   
    xlabel('Column');
    ylabel('Normalized Entropy');
    Emax = max(Entropy);
    Emin = min(Entropy);
    set(gca,'XTick',xtick);
    axis([1 n Emin*0.95 Emax*1.05]);
    title(strcat(num2str(n),'x',num2str(m)));
    
    savefilename = strcat(resultdir,'En',num2str(fileIdx),'r',...
        num2str(spIdx),'P.jpg');
    print(fig,savefilename,'-djpeg','-r72');
    
  end
  
  imagecat_en(fileIdx,Endir);
  close all;
  
end
