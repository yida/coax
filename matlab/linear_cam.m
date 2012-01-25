function linearImgI = linear_cam()

%bagDir = '~/bagfiles/2011-10-06-13-56-29/';
%bagDir = '~/bagfiles/2011-11-02-16-22-03-upstraight/';
%bagDir = '~/bagfiles/2011-11-02-16-25-44-updrunk/';
bagDir = '~/bagfiles/2011-11-02-16-30-06-midstraight/';
%bagDir = '~/bagfiles/2011-11-02-16-33-24-middrunk/';

imgDir = 'images/';
imgDir = strcat(bagDir,imgDir);

file = dir(strcat(imgDir,'*.jpg'));
nfile = size(file,1);

img = imread(strcat(imgDir,file(306).name));
[~,imgM,~] = size(img);

for reIdx = 10
disp(reIdx);
linearImgI = zeros(nfile,floor(imgM*reIdx/100));

t = CTimeleft(size(linearImgI, 1));
for fileIdx = 1:nfile
    t.timeleft();
fileName = file(fileIdx).name;

img = imread(strcat(imgDir,fileName));
imgI = rgb2gray(img);
imgI = imresize(imgI,reIdx/100);
[~,m] = size(imgI);
%imagesc(imgI);
%colormap(gray);

linearImgI(fileIdx,1:m) = mean(imgI,1);
end

imwrite(uint8(linearImgI),strcat(bagDir,num2str(reIdx),'P',fileName));

end  