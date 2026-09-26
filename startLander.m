function app = startLander()
%STARTLANDER Add project folders to the MATLAB path and launch the simulator.

    root = fileparts(mfilename('fullpath'));

    addpath(root);
    addpath(fullfile(root, 'physics'));
    addpath(fullfile(root, 'mpc'));
    addpath(fullfile(root, 'numerical'));
    addpath(fullfile(root, 'analysis'));
    addpath(fullfile(root, 'terrain'));

    app = LunarLanderMPC();
end