#!/bin/sh
docker pull devrt/simulator-index
docker run -ti --rm -v $(pwd):/work devrt/simulator-index
