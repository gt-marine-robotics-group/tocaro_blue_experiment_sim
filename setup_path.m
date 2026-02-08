function setup_path()
%SETUP_PATH Add src folder (and subfolders) to MATLAB path.

this_dir = fileparts(mfilename("fullpath"));
addpath(genpath(fullfile(this_dir, "src")));
end
