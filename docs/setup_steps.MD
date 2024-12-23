For mouseControl:
```
mkdir build
cd build
cmake .. 
make -j 4
./mouseControl
```

For opencv on mac:
```
# make sure you install opencv 
brew install opencv && brew link opencv
# confirm installation
pkg-config --modversion opencv4


# then 
mkdir build
cd build
cmake .. 
make -j 4
./webcam_stream

```

Make sure to camke in release mode for actual deployments

