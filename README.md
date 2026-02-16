# 英雄盲射定位方案

## 启动
```bash
./relocation_myservice.sh # 使用自己写的服务端
./relocation_myservice_no_trigger.sh # 不用 trigger 节点，等uart接口
```

## 可能存在的问题
1. TF树camera_init到aft_mapped必须有，但是odom到base_link可能需要删除，树就是
```
map -> odom           base_link -> base_footprint ......
        |                |
      camera_init -> aft_mapped
```
2. 现在会启动三个rviz，到时候看需要删掉哪个
    - 有FASTLIVO2、重定位（看点云配准效果）、TF树可视化