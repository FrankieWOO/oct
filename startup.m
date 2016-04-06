curPath = strsplit(pwd,{'\','/'});
curPath{1,end+1} = 'external/genpath_exclude/';
addpath(strjoin(curPath,'/'));
addpath(genpath_exclude(pwd,{'.git','.svn'}));
