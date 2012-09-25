function imagecat(fileIdx,BPdir)
  resultdir = 'results/';
  filename = strcat(resultdir,num2str(fileIdx),'r',num2str(5),'P.jpg');
  image = imread(filename);
  [m n k] = size(image);
  catImage = zeros(m*2,n*5,k);
  
  for rIdx = 1 : 5
      filename = strcat(resultdir,num2str(fileIdx),'r',num2str(rIdx*5),...
          'P.jpg');
      curImage = imread(filename);
      catImage(1:m,(rIdx-1)*n+1:rIdx*n,:) = curImage;
  end

  for rIdx = 6 : 10
      filename = strcat(resultdir,num2str(fileIdx),'r',num2str(rIdx*5),...
          'P.jpg');
      curImage = imread(filename);
      catImage(m+1:2*m,(rIdx-6)*n+1:(rIdx-5)*n,:) = curImage;
  end  
  
  catImage = uint8(catImage);
  catImagename = strcat(BPdir,num2str(fileIdx),'.jpg');
  imwrite(catImage,catImagename);
