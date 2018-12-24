## Compilation
g++ -o getKittiGT getKittiGTfromRTK.cpp  -std=c++11 -lstdc++fs -I /usr/include/eigen3

## Run
```sh
$ ./getKittiGT calib_dir data_dir start_index end_index
```

## Example
```sh
$ ./getKittiGT /home2/Kitti/raw_data/city/0002/2011_09_26_calib /home2/Kitti/raw_data/city/0002/2011_09_26_drive_0002_sync 0 76

```
