import os
import subprocess
import gzip

source_dir = "src/web/orig"
target_file = "src/web/website.h"


web_site_file = ""

def add_2_website_file(name, src):
    global web_site_file
    byteCnt = len(src)
    web_site_file += f"\n//File: {name}.gz size:{byteCnt}\n"
    web_site_file += f"#define {name.replace('.', '_')}_gz_len {byteCnt}\n"
    web_site_file += f"const uint8_t {name.replace('.', '_')}_gz[] = {{\n"
    cnt = 0
    for i, byte in enumerate(src):
        web_site_file += f" 0x{byte:02X}"
        if i < byteCnt - 1:
            web_site_file += ","
        cnt += 1
        if cnt == 16:
            web_site_file += "\n"
            cnt = 0
    web_site_file += "};\n\n"

def minify_css(css_code: str) -> str:
    try:
        proc = subprocess.run(
            ['cleancss.cmd', '--skip-rebase', '-o', '-', '-'],  # clean-css liest stdin, gibt minifiziert auf stdout
            input=css_code.encode('utf-8'),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True
        )
        return proc.stdout.decode('utf-8')
    except subprocess.CalledProcessError as e:
        print("clean-css error:", e.stderr.decode('utf-8'))
        return css_code

def minify_js(js_code: str) -> str:
    try:
        proc = subprocess.run(
            ['terser.cmd', '--compress', '--mangle'],
            input=js_code.encode('utf-8'),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True
        )
        return proc.stdout.decode('utf-8')
    except subprocess.CalledProcessError as e:
        print("terser error:", e.stderr.decode('utf-8'))
        return js_code

def minify_html(html_code: str) -> str:
    try:
        proc = subprocess.run(
            ['html-minifier-terser.cmd',
             '--collapse-whitespace',
             '--remove-comments',
             '--minify-css', 'true',
             '--minify-js', 'true'
             ],
            input=html_code.encode('utf-8'),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True
        )
        return proc.stdout.decode('utf-8')
    except subprocess.CalledProcessError as e:
        print("html-minifier-terser error:", e.stderr.decode('utf-8'))
        return html_code

def getFileContent(file):
    with open(file, 'r', encoding='utf-8') as f:
        content = f.read()
    return content

def minify_file(path, file):

    ext = os.path.splitext(file)[1].lower()
    if (ext == '.css') or  (ext == '.js') or (ext == '.html'):
        content = getFileContent(os.path.join(path,file))
    if ext == '.css':        
        minified = minify_css(content)
    elif ext == '.js':
        content = getFileContent(os.path.join(path,file))
        minified = minify_js(content)
    elif ext == '.html' or ext == '.htm':
        content = getFileContent(os.path.join(path,file))
        minified = minify_html(content)
    if (ext == '.css') or  (ext == '.js') or (ext == '.html'):
        minified_gzip = gzip.compress(minified.encode("utf-8"))
    else:
        print(f"fileextension not supported --> only zip it: {file}")
        with open(os.path.join(path,file), 'rb') as f:
            content = f.read()
        minified = content
        minified_gzip = gzip.compress(content)      
    print(f"file: {file} | original: {len(content)} | minimized: {len(minified)} | gzip: {len(minified_gzip)}")
    add_2_website_file(file, minified_gzip)


for root, dirs, files in os.walk(source_dir):
    for file in files:
        #minify_file(os.path.join(root, file))
        minify_file(root, file)

with open(target_file, "w", encoding="utf-8") as fout:
    fout.write(web_site_file)        

print("***** all files minimized *****")
