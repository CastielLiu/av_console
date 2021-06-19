#!/bin/sh

#run ./install sleep_time

pwd=$(readlink -f "$(dirname "$0")")   #fetch current file directory

if [ -n "$1" ]; then
    sleep $1
fi
cp ${pwd}/../../../devel/lib/av_console/av_console ${pwd}/app/

# general
name=av_console
exe=${pwd}/app/${name}
icon=${pwd}/icon/icon.jpg
newline="\n"

output=${pwd}/${name}.desktop

if [ -f "$output" ]; then
	rm "$output"
fi

echo "[Desktop Entry]" >> "$output"
echo "Type=Application" >> "$output"
echo "Exec=bash -i -c ${exe}" >> "$output"
echo "Name=${name}" >> "$output"
echo "GenericName=${name}" >> "$output"
echo "Icon=${icon}" >> "$output"
echo "Terminal=false" >> "$output"
echo "Categories=Development" >> "$output"

chmod a+x "$output"

if [ -f "~/Desktop/$output" ]; then
	rm "~/Desktop/$output"
fi

cp $output ~/Desktop/${name}.desktop

echo "install av_console to desktop complete."
