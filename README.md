# unizar-profiling-ros-pkg
The ros profiling package

Usage: 

`rosrun ros_profiling tx -s`

and

`rosrun ros_profiling rx -s -a ip_of_server_machine`

Both executable need a config file to be put on `~/.rospacket/config.yaml` or specified using `-f` option (use `-h` for details).

The format is for `tx`
```
topics:
 - name: topic_1
   period: 20
   size: 1000
   count: 1000
 - name: topic_2
   period: 300
   size: 64000
   count: 500
```

and 

```
topics:
 - name: topic_1
   type: 0
 - name: topic_2
   type: 2
```

where type specifies the type of connection (0 is TCP, 1 is TCP_NoDelay and 2 is UDP).
The `-a` option on the `rx` node, must specify the IP of the machine where the `tx` is running (should be a wired network).

For further information contact dantard@unizar.es.

