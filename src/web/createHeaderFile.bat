REM NOT WORKING AT THE MOMENT --> gzip is missing and creating website.h

delete .\stripped\*.html
delete .\stripped\*.js
delete .\stripped\*.css
copy .\orig\*.html .\stripped\
copy .\orig\*.js .\stripped\
copy .\orig\*.css .\stripped\
HtmlMinifier.exe ".\stripped"