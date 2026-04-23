function startup()
    % Domain-specific startup script
    [~, domainName] = fileparts(fileparts(mfilename('fullpath'))); % gets folder name
    fprintf('Initializing %s Environment...\n', domainName);
    
    % 1. Add this domain's files to the path
    domainRoot = fileparts(mfilename('fullpath'));
    addCleanPath(domainRoot);
    
    % 2. Add MEX C++ Extensions as a shared dependency
    projectRoot = fileparts(domainRoot);
    mexPath = fullfile(projectRoot, 'MEX_CPP_Extensions');
    if exist(mexPath, 'dir') && ~strcmp(domainRoot, mexPath)
        addCleanPath(mexPath);
        disp(' -> Linked MEX C++ Extensions');
    end
    
    disp('Environment Ready.');
end

function addCleanPath(rawDir)
    % Safely adds paths recursively while excluding restricted folders
    rawPathStr = genpath(rawDir);
    pathCells = regexp(rawPathStr, pathsep, 'split');
    exclusions = {'\.git', 'slprj', 'sfprj', 'Original_Archive', '\.SimulinkProject'};
    
    validPaths = {};
    for i = 1:length(pathCells)
        p = pathCells{i};
        if isempty(p), continue; end
        
        excludeFlag = false;
        for j = 1:length(exclusions)
            if ~isempty(regexp(p, exclusions{j}, 'once'))
                excludeFlag = true;
                break;
            end
        end
        if ~excludeFlag
            validPaths{end+1} = p; %#ok<AGROW>
        end
    end
    
    if ~isempty(validPaths)
        addpath(strjoin(validPaths, pathsep));
    end
end
