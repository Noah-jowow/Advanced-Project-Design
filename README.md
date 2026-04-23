# Advanced Project Design Architecture

Welcome to the Advanced Project Design workspace. This environment enforces strict architectural isolation between domains to prevent version smearing and to uphold extreme mathematical rigor.

## Architectural Overview

The repository is partitioned into strict, functional domains:
- **`Radar_Systems/`**: Tracking algorithms, directors, and UI applications.
- **`Aerodynamics_CFD/`**: Airfoil solvers and CFD processors.
- **`Propulsion/`**: Nozzle optimizers and related math.
- **`MEX_CPP_Extensions/`**: High-performance compiled C++ algorithms.

**Golden Rule:** Do not mix functional domains. Code generated for one domain must remain modular and isolated from the others. Dependencies between domains are strictly managed via path configuration.

---

## Environment Initialization (`startup.m`)

We utilize a **decentralized environment initialization**. 
Instead of a master path script, each domain folder contains its own `startup.m`. 

### How to Start Work
1. Launch MATLAB.
2. Navigate your Current Folder directly into the domain you want to work on (e.g., `cd Aerodynamics_CFD`).
3. The `startup.m` script will automatically execute.
4. The script will securely load the domain's subfolders and dynamically link the shared `MEX_CPP_Extensions` library, ignoring all backups and Git files.

---

## Creating a New Tool/Domain

When you start a new branch of research, create a new folder at the root level (e.g., `Orbital_Mechanics`). 

To ensure the new domain integrates perfectly into the architecture, copy the standard `startup.m` script into the new folder:

```matlab
function startup()
    % Domain-specific startup script
    [~, domainName] = fileparts(fileparts(mfilename('fullpath')));
    fprintf('Initializing %s Environment...\n', domainName);
    
    domainRoot = fileparts(mfilename('fullpath'));
    addCleanPath(domainRoot);
    
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
                excludeFlag = true; break;
            end
        end
        if ~excludeFlag, validPaths{end+1} = p; end
    end
    if ~isempty(validPaths), addpath(strjoin(validPaths, pathsep)); end
end
```

---

## C++ MEX Development

This workspace frequently utilizes C++ extensions to bypass MATLAB's computational bottlenecks.

All `.cpp` files and compiled `.mexw64` files must reside in the `MEX_CPP_Extensions` directory. 
When writing a MATLAB script that requires a MEX file, **DO NOT** call the MEX binary directly. Instead, utilize the robust `BoilerplateMEXWrapper.m` design pattern. 

### Wrapper Guidelines:
1. Copy `BoilerplateMEXWrapper.m` and rename it to your function.
2. Implement **Input Validation** using `narginchk` and `validateattributes` to prevent C++ segmentation faults.
3. Keep the dynamic **Auto-Compile Fallback** logic to ensure your code works across different operating systems.
4. Adhere to the Mathematical Rigor Guide—log any mathematical approximations directly in the wrapper's docstring.
