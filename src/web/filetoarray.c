/*
 * https://github.com/xreef/FileToArray
 *
 * to obtain executable < gcc -o filetoarray filetoarray.c
 * to create filearray < filetoarray "index.html.gz" > web_index.h
 *
 * Online converters also available
 *
 * FILE TO (CPP) GZIP BYTE ARRAY
 * https://www.mischianti.org/online-converter-file-to-cpp-gzip-byte-array-3/
 *
 * HEX ARRAY TO FILE
 * https://www.mischianti.org/online-converter-hex-array-to-file/
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char *argv[]){
    FILE * fp;
    char *buffer;
    long flen;
    char *fname;
    char pname[1024];

    if( argc == 2 ) {
        fname = argv[1];
        strcpy(pname, fname);
        char * dot = strchr(pname, '.');
        while(dot != NULL){
            *dot = '_';
            dot = strchr(pname, '.');
        }
    }else {
        printf("Filename not supplied\n");
        return 1;
    }

    fp = fopen(fname, "rb");
    fseek(fp, 0, SEEK_END);
    flen = ftell(fp);
    rewind(fp);

    buffer = (char *)malloc((flen+1)*sizeof(char));
    fread(buffer, flen, 1, fp);
    fclose(fp);

    printf("\n//File: %s, Size: %lu\n",fname, flen);
    printf("#define %s_len %lu\n", pname, flen);
    printf("const uint8_t %s[] PROGMEM = {\n", pname);
    long i;
    for(i=0;i<flen;i++){
        printf(" 0x%02X", (uint8_t)buffer[i]);
        if(i<(flen - 1)){
            printf(",");
        }
        if((i%16)==15){
            printf("\n");
        }
    }
    printf("\n};\n\n");
    free(buffer);
    return 0;
}