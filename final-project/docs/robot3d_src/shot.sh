#!/bin/bash
# shot.sh page.html out.png [width height] [hash]
CH=~/.cache/ms-playwright/chromium_headless_shell-1248/chrome-headless-shell-linux64/chrome-headless-shell
$CH --headless --no-sandbox --use-angle=swiftshader --enable-unsafe-swiftshader --hide-scrollbars \
  --window-size=${3:-1200},${4:-800} --virtual-time-budget=${6:-15000} --screenshot="$2" "file://$1${5:+#$5}" 2>&1 | grep -iv "dbus\|fontconfig\|gpu\|sandbox\|bytes written" | cut -c1-300 | head -20
