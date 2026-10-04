import os
import subprocess
import gzip
import shutil

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

def run_minifier(cmd_list, input_code):
    """Helper to run minifiers with shell=True on Windows for .cmd files."""
    try:
        # Check if the command exists in PATH
        if not shutil.which(cmd_list[0]):
            return input_code
            
        proc = subprocess.run(
            cmd_list,
            input=input_code.encode('utf-8'),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=(os.name == 'nt'), # Essential for .cmd files on Windows
            check=True
        )
        return proc.stdout.decode('utf-8')
    except (subprocess.CalledProcessError, FileNotFoundError):
        return input_code

def minify_css(css_code):
    return run_minifier(['cleancss.cmd', '--skip-rebase'], css_code)

def minify_js(js_code):
    return run_minifier(['terser.cmd', '--compress', '--mangle'], js_code)

def minify_html(html_code):
    return run_minifier(['html-minifier-terser.cmd', '--collapse-whitespace', '--remove-comments', '--minify-css', 'true', '--minify-js', 'true'], html_code)

def minify_file(path, file):
    ext = os.path.splitext(file)[1].lower()
    full_path = os.path.join(path, file)
    
    if ext in ['.css', '.js', '.html', '.htm']:
        with open(full_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        if ext == '.css':
            minified = minify_css(content)
        elif ext == '.js':
            minified = minify_js(content)
        else:
            minified = minify_html(content)
            
        final_data = gzip.compress(minified.encode("utf-8"))
        print(f"file: {file} | original: {len(content)} | minimized: {len(minified)} | gzip: {len(final_data)}")
    else:
        # For non-text files like favicon.ico
        with open(full_path, 'rb') as f:
            content = f.read()
        final_data = gzip.compress(content)
        print(f"file: {file} | original: {len(content)} | gzip: {len(final_data)}")
        
    add_2_website_file(file, final_data)

# Main execution
if os.path.exists(source_dir):
    for root, dirs, files in os.walk(source_dir):
        for file in files:
            minify_file(root, file)

    with open(target_file, "w", encoding="utf-8") as fout:
        fout.write(web_site_file)
    print("***** website.h regenerated successfully *****")
else:
    print(f"Error: Source directory {source_dir} not found.")