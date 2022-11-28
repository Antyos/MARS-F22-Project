path = genpath('robotarium-matlab-simulator');
if(isempty(path))
    disp('WARNING: Cannot find robotarium directory.  This script should be run from the base directory.')
else
    addpath(path)
end

addpath('./')