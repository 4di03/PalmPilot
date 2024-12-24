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

# sample usage of hand tracking:
build/openpose --proto=data/models/pose.prototxt --model=data/models/pose_iter_102000.caffemodel --image=data/hand_sample.png --dataset=HAND

```


Make sure to cmake in release mode for actual deployments




