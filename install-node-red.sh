#!/bin/bash

echo "TARGET ARCHITECTURE: ${TARGETARCH}"

wget -O installer.sh \
  https://raw.githubusercontent.com/node-red/linux-installers/master/deb/update-nodejs-and-nodered

if [[ ${TARGETARCH} = arm* ]]; then
  echo "BUILDING FOR ARM"

  bash ./installer.sh --confirm-root --confirm-install --no-init \
    && rm installer.sh

  pushd /root/.node-red

  npm install zeromq@^6.0.0
else
  echo "NOT BUILDING FOR ARM"

  bash ./installer.sh --confirm-root --confirm-install --no-init --skip-pi \
    && rm installer.sh

  pushd /root/.node-red

  npm install zeromq@^6.0.0
fi

npm install \
  node-red-dashboard \
  node-red-contrib-modbus \
  @node-red-contrib-themes/theme-collection

popd
