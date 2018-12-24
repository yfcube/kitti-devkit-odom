## Compilation
```sh
$ g++ -o kitti_poses kitti_poses_from_mapping_poses.cpp -std=c++11 -lstdc++fs -I /usr/include/eigen3
```

## Run
```sh
$ ./kitti_poses mapping_poses_file
```

## Example
```sh
$ ./kitti_poses /home2/Kitti/Result/mapping/11/final_mapping_pose.txt
```