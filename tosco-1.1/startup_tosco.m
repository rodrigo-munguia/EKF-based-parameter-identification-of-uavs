disp('Tosco Toolbox 1.1   Rodrigo Munguía 2015')

toscopath = fileparts( mfilename('fullpath') );

commonpath = fullfile(toscopath, 'common');
if exist(commonpath,'dir')
    addpath(commonpath);  
end


visionpath = fullfile(toscopath, 'vision');
if exist(visionpath,'dir')
    addpath(visionpath);  
end

plottingpath = fullfile(toscopath, 'plotting');
if exist(plottingpath,'dir')
    addpath(plottingpath);  
end


clear toscopath commonpath visionpath plottingpath
