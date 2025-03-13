REM uses minify.exe from https://github.com/tdewolff/minify
REM uses filetoarray.exe from https://github.com/xreef/FileToArray
REm uses gzip from https://gnuwin32.sourceforge.net/packages/gzip.htm
REM uses cygwin1.dll from https://www.cygwin.com/
REM due to copyright questions this files must be copied from their website !!!
REM 
REM copies the original html/css/js files while minimizing them
REM gzips the copied files
REM copies the compressed files to \data\ directory for SPIFFS
REM creates the file website.h
del .\stripped\*.html
del .\stripped\*.js
del .\stripped\*.css
del .\stripped\*.gz
del .\stripped\orig\*.* /Q
rem CALL HtmlMinifier.exe ".\stripped\"
call minify -v -s -r -o .\stripped\ .\orig\
move .\stripped\orig\* .\stripped
rmdir .\stripped\orig
FOR  %%y IN (".\stripped\*.*") DO (gzip -k %%y)
copy .\stripped\*.gz ..\..\data\
cd stripped
del tmp.txt
FOR  %%y IN ("*.gz") DO (..\filetoarray %%y >> tmp.txt)
del *.gz
cd ..
move stripped\tmp.txt website.h
