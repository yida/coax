function run_vision_test(machine)

if nargin == 0
    machine = '';
end
%% Get Data Dir
DataDir = '~/bagfiles';

dirData = dir(DataDir);
for dirIdx = 1 : numel(dirData)
    % remove . and ..
    if (strcmp(dirData(dirIdx).name,'.') == 1) || ...
        (strcmp(dirData(dirIdx).name,'..') == 1)
        continue;
    end
    % remove files
    if (dirData(dirIdx).isdir == 1)
        bagName = dirData(dirIdx).name;
    else
        continue;
    end
	disp(bagName);
    bagDir = strcat(DataDir,'/',bagName);
    cd(bagDir);
    addpath('../../coax_vision/');
    if (strcmp(machine,'')==1)
        vision_test(bagDir);
    elseif (strcmp(machine,'fig')==1)
		vision_test_fig(bagDir);
	elseif (strcmp(machine,'date')==1)
		vision_test_date(bagDir);
	elseif (strcmp(machine,'apricot')==1)
		vision_test_apricot(bagDir);
	elseif (strcmp(machine,'prune')==1)
		vision_test_prune(bagDir);
	end
    cd('..');
    addpath('../coax_vision/');
end
