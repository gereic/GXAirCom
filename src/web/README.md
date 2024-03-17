# Generating Web Content

Web content files are stored in the /src/web/orig directory. When changes are made they need to be processed for uploading
onto a device. Processing includes:

1) Minifying the files in /src/web/orig and storing them in /src/web/stripped
2) Compressing the files in /src/web/stripped and storing them in /data
3) Loading the compressed files into arrays in the website.h file to support devices that do not want to use SPIFFs
4) Updating the device by compiling and installing a new firmware (for devices that don't use SPIFFS), or uploading a 
   new Filesystem Image for devices that do support SPIFFS.

## Installing Prerequisites

The process of building the web content relies on two tools.

1) Minify - https://github.com/tdewolff/minify
2) FileToArray - https://github.com/xreef/FileToArray

### Windows, Linux

The binaries for these tools are included in this repo. You should be able to use
them without needing to install anything.

### Mac

Install Minify using Homebrew

```[cmd]
brew install tdewolff/tap/minify
```

Compile the FileToArray tool directly

```[cmd]
gcc -o filetoarray filetoarray.c
```

## Running

Once you have the prerequisites installed you can:

1) Run .\createHeaderFile.bat (Windows), or .\createHeaderFile.sh (Linux, MacOS)
2) Update device firmware
3) Update device Filesystem Image