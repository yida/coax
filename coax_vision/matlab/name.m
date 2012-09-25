file = dir('*.jpg');
nfile = size(file,1);

for i = 1 : nfile
    disp(file(i).name);
    filelen = length(file(i).name);
    filename = file(i).name(1:filelen-4);
    if (length(filename) == 1)
        filename = strcat('out000',filename);
    elseif (length(filename) == 2)
        filename = strcat('out00',filename);
    elseif (length(filename) == 3)
        filename = strcat('out0',filename);
    else
        filename = strcat('out',filename);
    end
    filename = strcat(filename,'.jpg');
    movefile(file(i).name,filename);
end