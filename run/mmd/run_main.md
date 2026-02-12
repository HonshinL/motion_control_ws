main()
├── rclcpp::init()
├── ZmcController构造函数
│   ├── 创建发布者 (motion_status_pub_)
│   ├── 创建订阅者 (object_position_sub_)
│   ├── 创建服务 (convert_dxf_to_xml_service_)
│   └── 创建Action服务器 (move_to_position_action_server_)
├── zmc_controller->start()
│   ├── 连接ZMotion控制器
│   └── 启动定时器 (startPublishing)
├── rclcpp::spin() [主循环]
│   ├── timer_callback() [每20ms]
│   ├── handleObjectPosition() [消息到达时]
│   ├── handleConvertDxfToXml() [服务调用时]
│   └── handleMoveToPosition() [Action请求时]
├── zmc_controller->stop()
│   ├── 停止定时器 (stopPublishing)
│   └── 断开控制器连接
└── rclcpp::shutdown()