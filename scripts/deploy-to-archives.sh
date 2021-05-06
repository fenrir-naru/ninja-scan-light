#!/bin/bash
# install ghcp
(curl -fL -o /tmp/ghcp.zip https://github.com/int128/ghcp/releases/download/v1.8.0/ghcp_linux_amd64.zip \
  && unzip -d /tmp /tmp/ghcp.zip)
# push artifacts
COMMIT_MESSAGE="build by Travis-CI, `ruby -e 'require "time"; print Time::parse(\`git log --format="%cd" -n 1\`.chomp).strftime("%Y%m%d_%H%M%S")'`"
(cd /tmp/artifacts \
  && /tmp/ghcp commit -u fenrir-naru -r archives -m "${COMMIT_MESSAGE}" --token=$GITHUB_PERSONAL_TOKEN \
    $(find . -type f | sed -n 's|^\./||p') )