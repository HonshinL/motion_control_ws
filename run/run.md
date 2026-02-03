


```
g++ src/main.cpp src/zmcaux.cpp \
    -I include \
    -L lib \
    -lzmotion -lpthread \
    -o zmotion_test
```

```
LD_LIBRARY_PATH=./lib ./zmotion_test 192.168.0.11
LD_LIBRARY_PATH=./lib ./get_controller_info 192.168.0.11
```

ros2 run zmotion_driver zmotion_test