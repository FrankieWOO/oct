Default output dir of mex file compiled from C is m-files, however it should be put into model_maccepa_d2 or _d3 according to the U dim.

Makefile may need to be modifiled before use. In windows, use mexext.bat instead of mexext; if you have installed MikTex, mex may refer to a Tex program, so it's better to use mex.bat as well.

I compiled mex file with the makefile using Cygwin in windows.