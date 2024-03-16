url="https://ziglang.org/download/"

webcontent=`curl -s $url`
current_downloaded=`ls ~/langs/ | grep zig`
download_dir=~/Downloads/
lang_dir=~/langs/
ziglocfile=~/.config/zig/zigloc

package_name_regex="(zig-linux-x86_64-0\.12\.0-dev\.[0-9a-z\+]+)\.tar\.xz"
distro_url_regex="<a href=(https:\/\/ziglang.org\/builds\/zig-linux-x86_64-0\.12\.0-dev\.[0-9a-z\+]+\.tar\.xz)>"

# Download nightly
if [[ $webcontent =~ $distro_url_regex ]]
then
    download_url="${BASH_REMATCH[1]}"
    if [[ $download_url =~ $package_name_regex ]]
    then
        current_nightly_name="${BASH_REMATCH[1]}"
        if [[ $current_nightly_name != $current_downloaded ]] 
        then
            wget -q -P $download_dir $download_url
            tar -xf "${download_dir}${current_nightly_name}.tar.xz" -C $lang_dir
            echo 'export PATH="$PATH:$lang_dir$current_nightly_name"' > $ziglocfile
            rm -f ${download_dir}${current_nightly_name}
        else
            echo "Zig nightly check: Already up to date."
        fi
    else
        echo "Zig nightly check: Package name not found."
    fi
else
    echo "Zig nightly check: Distro url not found."
fi
