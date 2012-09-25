function [] = vision_BP(img,fileIdx,method,part)

% img : input grayscale image
% fileIdx : number of files for save
% method : mean , median
% part : 'whole' image; 'upper' half image; 'lower' half image;

% Get original size of image and trunct it
[imgN,~] = size(img);
if (strcmp(part,'upper') == 1)
    img = img(1:imgN/2,:);
elseif (strcmp(part,'lower') == 1)
    img = img(imgN/2+1:imgN,:);
end

% Create output dirs
outDir = strcat('BP_',method,'_',part,'/');
outDirRaw = strcat(outDir,'raw/');

if isdir(outDir) == 0
  mkdir(outDir);
end

if isdir(outDirRaw) == 0
  mkdir(outDirRaw);
end

% Drease resolution
for spIdx = 5:5:50
    sfilename = strcat(outDirRaw,num2str(fileIdx),'r',...
        num2str(spIdx),'P.jpg');
    if (exist(sfilename,'file') == 2)
        continue;
    end
    BP = imresize(img,spIdx/100);
    [m n] = size(BP);
    subplot(2,1,1);
    imagesc(BP);
    colormap(gray);
    BPC = bright_pixels(BP,max(1,round(10*spIdx/100)),method);
    x = 1 : n;
    xtick = round(linspace(1,n,5));
    subplot(2,1,2);
    plot(x,BPC);
    xlabel('Column');
    ylabel(strcat('BP-',method));
    set(gca,'XTick',xtick);
    axis([1 n min(BPC)*0.95 max(BPC)*1.05]);
    title(strcat(num2str(n),'x',num2str(m),' ',part));


    print(sfilename,'-djpeg','-r72');
end

% connect images

catImagename = strcat(outDir,num2str(fileIdx),'.jpg');
if (exist(catImagename,'file')~=2)
    inputname = strcat(outDirRaw,num2str(fileIdx),'r',num2str(5),'P.jpg');
    Image = imread(inputname);
    [Im In Ik] = size(Image);
    catImage = zeros(Im*2,In*5,Ik);
  
    for rIdx = 1 : 5
        inputname = strcat(outDirRaw,num2str(fileIdx),'r',num2str(rIdx*5),'P.jpg');
        curImage = imread(inputname);
        catImage(1:Im,(rIdx-1)*In+1:rIdx*In,:) = curImage;
    end

    for rIdx = 6 : 10
        inputname = strcat(outDirRaw,num2str(fileIdx),'r',num2str(rIdx*5),'P.jpg');
        curImage = imread(inputname);
        catImage(Im+1:2*Im,(rIdx-6)*In+1:(rIdx-5)*In,:) = curImage;
    end  
    catImage = uint8(catImage);
    imwrite(catImage,catImagename);
end


