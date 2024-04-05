#!/bin/bash

# requires minify from https://github.com/tdewolff/minify
# on Debian: apt install minify

set -e

TMP=$(mktemp)
trap 'rm -f "${TMP}"' EXIT

rm -f stripped/*.gz stripped/*.{css,js,html}
for i in orig/*.{css,js,html}; do
    si=${i/orig/stripped/}
    ./minify \
        --js-keep-var-names \
        --html-keep-default-attrvals \
        --js-precision 0 \
        "${i}" > "${si}"
    gzip -k "${si}"
done

find ../../data -type f -delete

#cp stripped/*.html ../../data/
#cp stripped/*.js ../../data/
#cp stripped/*.css ../../data/

pushd stripped >/dev/null
for i in *.gz; do
    ../filetoarray "${i}" >> "${TMP}"    
done
popd >/dev/null

cp stripped/*.gz ../../data/
rm -f stripped/*.gz

mv "${TMP}" "website.h"
