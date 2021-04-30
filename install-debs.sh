#!/usr/bin/env bash
function announce() {
    echo "**************************************************"
    echo -e "$@"
    echo "**************************************************"
}

OLD_PWD=$PWD
THIS_DIR="$(dirname $(readlink -f $0))"
cd $THIS_DIR

if [[ ! -d debs ]];
then
    announce "Error: debs directory doesn't exist. Make sure you create it and paste the debian packages from the portal."
    return 1
fi

if ! $(ls debs/*.deb 1>/dev/null 2>&1);
then
    announce "Error: debs directory doesn't contain the necessary debian pacakges. Make sure you create it and paste the debian packages from the portal."
    return 1
fi

announce "Installing SLAMcore debian packages: $(ls debs/*.deb)"
sudo apt install -y debs/*.deb
cd $OLD_PWD
