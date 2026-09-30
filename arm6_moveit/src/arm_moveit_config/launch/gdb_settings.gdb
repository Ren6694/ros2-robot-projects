# =============================================================================
#  gdb_settings.gdb —— move_group 在 debug:=true 时加载的 gdb 命令脚本
#
#  generate_move_group_launch() 无条件把 <config_pkg>/launch/gdb_settings.gdb
#  作为 `gdb -x <此文件>` 的脚本传入。文件不存在时 debug:=false 也照样能跑，
#  但 debug:=true 会在 gdb 启动阶段就失败 —— 所以这个"看起来没用"的文件必须装。
#
#  每条命令为什么在这：
#    set confirm off     非交互批处理下不询问"要不要 quit"，否则脚本会卡住
#    set pagination off  关掉分页，日志能一次性 tee 到文件里（阶段一 GDB 同款坑）
#    handle SIGPIPE nostop  MoveIt/rviz 之间断链时 DDS 会抛 SIGPIPE，
#                           gdb 默认停下，看起来像"程序崩溃"，其实是正常关闭
#    run                 启动被调试的节点
#    thread apply all bt 崩溃后打印**所有线程**回溯：move_group 是多线程的
#                         （规划线程、场景监控线程、执行监控线程），
#                         只看当前线程往往看不到真正出错的那一个
#    quit                批处理结束后退出 gdb，让 launch 正常收尾
# =============================================================================
set confirm off
set pagination off
handle SIGPIPE nostop noprint pass
run
thread apply all bt
quit
