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
3. 注意改话题，yaml里有几个话题参数
4. 如果要可视化，注意rviz里也要改相应话题、坐标系的参数和名字
    - 发布经过配准的点云，也就是更改LIVO2发出的`/pointcloud_registered`的点云的坐标系，现在改的是odom，话题为：`/registration/pointcloud_registered`
    - 目的：当漂移发生时，配准把odom和camera_init一起拉回来，如果后面再发生漂移，可以看到橙色点云变化
    - 点云持续显示时间是0.5s，可以再重定位节点里的RViz里改
5. **编译创建软链接--symlink-install**
```bash
colcon build --symlink-install
```
6. 我猜测FAST-LIVO2的漂移并不严重，后续回去可以直接摇人晃动车去看里程计是否有问题
7. 后续分支：
    - 改FAST-LIVO2源码，名字：`fix_LIVO2`
    - 增加一版只用FAST-LIVO2，不使用重定位，名字：`only_LIVO`
## 进度
- 2026.3.3：完成上车第一版：LIVO2 + Qautro + small-gicp，效果很好，但是非常消耗资源，经过优化后，CPU有一半多的核心占用率再30%左右，其他较高再90%左右
- 2026.3.7：完成英雄专属版本的长理测试：分支`no_LIVO2_V1.2`，结构是quatro+small-gicp，无LIVO2，效果很好，没有什么需要修改的
    - 可以删除一些注释，比如registration节点换用无调试信息的两个函数
- 2026.3.8：完成哨兵重定位方案长理测试，分支`main`FASTLIVO2+Quatro+smallgicp，效果极好，配准良好，base_link很准确，运行TRAKING后CPU占用依然可以接受，但是有几个小问题
    - TF中：LIVO2部分的camera_init和aft_mapped的TF坐标系无法链接到主树树上,**已经解决,TF树完好**
    - 为了节省资源，设计为离散重定位触发：初始和过起伏之后触发，无需traking。**注意重定位TF必须连续发送，不然TF树会断开，因为TF的buffer默认是10s**
    - **下一次着重测试：我方建图位姿固定，换为对方启动会不会漂移**