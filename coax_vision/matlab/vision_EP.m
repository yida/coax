function [] = vision_EP(img,fileIdx,part)

% img : input grayscale image
% fileIdx : number of files for save
% part : 'whole' image; 'upper' half image; 'lower' half image;

% Get original size of image and trunct it
[imgN,~] = size(img);
if (strcmp(part,'upper') == 1)
    img = img(1:imgN/2,:);
elseif (strcmp(part,'lower') == 1)
    img = img(imgN/2+1:imgN,:);
end

% Create output dirs
outDir = strcat('EP','_',part,'/');
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
    if (exist(sfilename,'file')==2)
        continue;
    end
    BP = imresize(img,spIdx/100);
    [m n] = size(BP);
    subplot(2,1,1);
    imagesc(BP);
    colormap(gray)
    
    Entropy = zeros(n,1);
    pshift = round(10*spIdx/100);
    for col=1:n
        leftpnt = max(1,col-pshift);
        rightpnt = min(n,col+pshift);
        slice = BP(:,leftpnt:rightpnt);
        Entropy(col,1) = entropy(slice);
    end
    
    x = 1 : n;
    xtick = round(linspace(1,n,5));
    subplot(2,1,2);
    plot(x,Entropy);
    xlabel('Column');
    ylabel('Entropy');
    set(gca,'XTick',xtick);
    axis([1 n min(Entropy)*0.95 max(Entropy)*1.05]);
    title(strcat(num2str(n),'x',num2str(m),' ',part));
    
    print(sfilename,'-djpeg','-r72');
end

% connect images

catImagename = strcat(outDir,num2str(fileIdx),'.jpg');
if (exist(catImagename,'file')~=2)
    filename = strcat(outDirRaw,num2str(fileIdx),'r',num2str(5),'P.jpg');
    image = imread(filename);
    [m n k] = size(image);
    catImage = zeros(m*2,n*5,k);
  
    for rIdx = 1 : 5
        filename = strcat(outDirRaw,num2str(fileIdx),'r',num2str(rIdx*5),'P.jpg');
        curImage = imread(filename);
        catImage(1:m,(rIdx-1)*n+1:rIdx*n,:) = curImage;
    end

    for rIdx = 6 : 10
        filename = strcat(outDirRaw,num2str(fileIdx),'r',num2str(rIdx*5),'P.jpg');
        curImage = imread(filename);
        catImage(m+1:2*m,(rIdx-6)*n+1:(rIdx-5)*n,:) = curImage;
    end  
  
    catImage = uint8(catImage);

    imwrite(catImage,catImagename);
end



