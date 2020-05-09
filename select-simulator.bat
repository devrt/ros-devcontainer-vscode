@echo off
docker run -ti --rm -v %cd%:/work devrt/simulator-index
pause