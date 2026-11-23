"""Inline three.js + addons (as data: URLs in an importmap) and the packed LDraw parts into one HTML file."""
import base64, sys
T = "/tmp/three/"
def durl(path):
    return "data:text/javascript;base64," + base64.b64encode(open(path, "rb").read()).decode()
src, mpd, out = sys.argv[1], sys.argv[2], sys.argv[3]
asm = sys.argv[4] if len(sys.argv) > 4 else None
html = open(src).read()
imap = ('<script type="importmap">{"imports":{"three":"%s","three/addons/LDrawLoader.js":"%s",'
        '"three/addons/OrbitControls.js":"%s"}}</script>') % (durl(T + "three.module.min.js"), durl(T + "LDrawLoader.js"),
                                                           durl(T + "OrbitControls.js"))
mpdtxt = open(mpd).read().replace("</script", "<\\/script")
html = html.replace("<!--IMPORTMAP-->", imap).replace("/*MPD*/", mpdtxt)
if asm: html = html.replace("/*ASM*/", open(asm).read())
open(out, "w").write(html)
print(out, round(len(html) / 1e6, 2), "MB")
