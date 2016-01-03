#!/bin/bash
sudo cp ad-hocset.sh /etc/init.d/
sudo update-rc.d ad-hocset.sh defaults 100

# to remove from startup
#sudo update-rc.d -f ad-hocset.sh remove
