function [] = vision_BP_re(~,~,~)

for fileIdx = 1 : 2055
% connect images
outDir = '';
outDirRaw = './raw/';
disp(fileIdx);

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


end