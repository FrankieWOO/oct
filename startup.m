addpath('external/genpath_exclude');
dir='.'         ;try,addpath(genpath_exclude(dir,{'.svn','doc','external'})),catch,warning(['Directory ',dir,' not found.']),end
