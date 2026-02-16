```bash
ros2 run trigger_registration trigger_hero_node
```

看总线程树数目
```bash
top
```
后按大写H， 看Threads后running的数目
running的超过硬线程数，那么性能才会下降

看ROS2线程数目
```bash
# 找到 ROS 节点的 PID
ps aux | grep <你的节点名>
# 查看该节点的线程数
ps -o nlwp= -p <PID>
```