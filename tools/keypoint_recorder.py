#!/usr/bin/env python3

# 关键点录制工具，需在已加载 ROS 工作空间的交互终端中手动启动；默认 run.launch 不启动它。
# 订阅 nav_msgs/Odometry，仅在按键时保存最新有效位置；输出固定在本脚本旁的 keypoint.yaml。
# 终端序号从 1 开始，YAML 的 waypoint 索引从 0 开始，供 FSM 的 navi_mode=2 读取。
import argparse
import math
import os
import select
import sys
import termios
import tty

try:
    import rospy
    from nav_msgs.msg import Odometry
except ImportError as exc:
    print("Failed to import ROS Python modules: {}".format(exc), file=sys.stderr)
    print("Source your ROS workspace first, for example:", file=sys.stderr)
    print("  source devel/setup.bash", file=sys.stderr)
    sys.exit(2)


DEFAULT_OUTPUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "keypoint.yaml")


# 移除 ROS 重映射参数，避免 argparse 把 __name:= 等参数当作未知命令行选项。
def strip_ros_args(argv):
    return [arg for arg in argv if ":=" not in arg and not arg.startswith("__")]


# 固定小数精度后清理尾零，并统一负零表示，使 YAML 的数值便于阅读和比较。
def format_float(value):
    text = "{:.6f}".format(float(value)).rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


# 先写同目录临时文件，再使用 os.replace 替换目标，减少中断时留下半份 YAML 的风险。
def atomic_write(path, content):
    directory = os.path.dirname(path)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)

    tmp_path = path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        handle.write(content)
    os.replace(tmp_path, path)


# 维护最新里程计与内存中的关键点列表；回调只更新观测，键盘命令负责修改列表和保存。
class KeypointRecorder:
    # 保存订阅话题和输出路径，以队列长度 1 接收最新位姿，避免积压旧采样。
    def __init__(self, args):
        self.odom_topic = args.odom
        self.output_path = DEFAULT_OUTPUT
        self.latest_odom = None
        self.waypoints = []

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

    # 缓存最新 Odometry；这里不会自动记录关键点。
    def odom_callback(self, msg):
        self.latest_odom = msg

    # 提取当前位置并拒绝 NaN/Inf，无有效观测时返回 None。
    def current_point(self):
        if self.latest_odom is None:
            return None

        position = self.latest_odom.pose.pose.position
        point = (float(position.x), float(position.y), float(position.z))
        return point if all(math.isfinite(value) for value in point) else None

    # 把当前有效位置追加到列表，保留录制时的世界系高度。
    def record_current(self):
        point = self.current_point()
        if point is None:
            rospy.logwarn("No valid odometry yet; cannot record waypoint.")
            return

        self.waypoints.append(point)
        rospy.loginfo(
            "Recorded waypoint %d: [%.3f, %.3f, %.3f]",
            len(self.waypoints),
            point[0],
            point[1],
            point[2],
        )

    # 按用户从 1 开始的序号覆盖已有点，内部数组索引减 1。
    def replace_current(self, index):
        if index < 1 or index > len(self.waypoints):
            rospy.logwarn("Invalid waypoint index %d. Valid range: 1-%d.", index, len(self.waypoints))
            return

        point = self.current_point()
        if point is None:
            rospy.logwarn("No valid odometry yet; cannot replace waypoint.")
            return

        self.waypoints[index - 1] = point
        rospy.loginfo(
            "Replaced waypoint %d: [%.3f, %.3f, %.3f]",
            index,
            point[0],
            point[1],
            point[2],
        )

    # 验证序号后删除指定点；仅修改内存，持久化由 save() 或退出命令完成。
    def delete_waypoint(self, index):
        if index < 1 or index > len(self.waypoints):
            rospy.logwarn("Invalid waypoint index %d. Valid range: 1-%d.", index, len(self.waypoints))
            return

        point = self.waypoints.pop(index - 1)
        rospy.loginfo(
            "Deleted waypoint %d: [%.3f, %.3f, %.3f]",
            index,
            point[0],
            point[1],
            point[2],
        )

    # 撤销最后一个已录制点，空列表时不执行删除。
    def undo_last(self):
        if not self.waypoints:
            rospy.logwarn("No waypoint to undo.")
            return

        point = self.waypoints.pop()
        rospy.loginfo(
            "Removed waypoint %d: [%.3f, %.3f, %.3f]",
            len(self.waypoints) + 1,
            point[0],
            point[1],
            point[2],
        )

    # 按面向用户的 1 起始序号打印列表，不改变关键点顺序。
    def list_waypoints(self):
        if not self.waypoints:
            rospy.loginfo("No recorded waypoints.")
            return

        rospy.loginfo("Recorded waypoints:")
        for index, point in enumerate(self.waypoints, 1):
            rospy.loginfo("  %02d: [%.3f, %.3f, %.3f]", index, point[0], point[1], point[2])

    # 按 FSM 约定生成 fsm/waypoint_num 和 waypointN_x/y/z 键，N 从 0 开始。
    def build_yaml(self):
        lines = ["fsm:", "  waypoint_num: {}".format(len(self.waypoints))]
        for index, point in enumerate(self.waypoints):
            lines.append("  waypoint{}_x: {}".format(index, format_float(point[0])))
            lines.append("  waypoint{}_y: {}".format(index, format_float(point[1])))
            lines.append("  waypoint{}_z: {}".format(index, format_float(point[2])))
        return "\n".join(lines) + "\n"

    # 调用原子替换写出完整 YAML，输出位置不随当前工作目录变化。
    def save(self):
        atomic_write(self.output_path, self.build_yaml())
        rospy.loginfo("Saved %d waypoint(s) to %s", len(self.waypoints), self.output_path)

    # 显示键盘命令、当前话题和输出位置，便于现场确认录制配置。
    def print_help(self):
        print("")
        print("keypoint_recorder")
        print("  odom topic : {}".format(self.odom_topic))
        print("  output     : {}".format(self.output_path))
        print("")
        print("Keyboard commands:")
        print("  Enter / Space / a : record current odom position")
        print("  r                 : replace an existing waypoint")
        print("  d                 : delete an existing waypoint")
        print("  u                 : undo last waypoint")
        print("  l                 : list waypoints")
        print("  s                 : save YAML")
        print("  h / ?             : show this help")
        print("  q                 : save and quit")
        print("")

    # 临时恢复终端行输入以读取序号，finally 中重新启用逐键输入模式。
    def prompt_index(self, old_settings, action):
        if not self.waypoints:
            rospy.logwarn("No waypoint to %s.", action)
            return None

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        try:
            text = input("{} waypoint index (1-{}): ".format(action.capitalize(), len(self.waypoints))).strip()
            if not text:
                print("{} cancelled.".format(action.capitalize()))
                return None
            return int(text)
        except ValueError:
            rospy.logwarn("Invalid waypoint index: %s", text)
            return None
        finally:
            tty.setcbreak(sys.stdin.fileno())

    # 要求交互 TTY，以 20 Hz 检查键盘命令；finally 恢复终端属性，避免异常后终端无回显。
    def run(self):
        self.print_help()
        rospy.loginfo("Waiting for odometry on %s ...", self.odom_topic)

        if not sys.stdin.isatty():
            rospy.logerr("stdin is not a TTY; keyboard control is required.")
            return

        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        rate = rospy.Rate(20)

        try:
            while not rospy.is_shutdown():
                ready, _, _ = select.select([sys.stdin], [], [], 0.0)
                if ready:
                    ch = sys.stdin.read(1)
                    if ch in ("\n", "\r", " ", "a", "A"):
                        self.record_current()
                    elif ch in ("r", "R"):
                        index = self.prompt_index(old_settings, "replace")
                        if index is not None:
                            self.replace_current(index)
                    elif ch in ("d", "D"):
                        index = self.prompt_index(old_settings, "delete")
                        if index is not None:
                            self.delete_waypoint(index)
                    elif ch in ("u", "U"):
                        self.undo_last()
                    elif ch in ("l", "L"):
                        self.list_waypoints()
                    elif ch in ("s", "S"):
                        self.save()
                    elif ch in ("h", "H", "?"):
                        self.print_help()
                    elif ch in ("q", "Q"):
                        self.save()
                        rospy.signal_shutdown("user requested quit")
                        break
                    elif ch == "\x03":
                        rospy.signal_shutdown("keyboard interrupt")
                        break
                rate.sleep()
        except (KeyboardInterrupt, rospy.ROSInterruptException):
            rospy.signal_shutdown("interrupted")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


# 声明 --odom 订阅选项；默认真机话题 /LIO/odom_vehicle，仿真时需显式指定机身话题。
def build_arg_parser():
    parser = argparse.ArgumentParser(description="Record odometry positions to tools/keypoint.yaml.")
    # 命令行参数 --odom：只选择录制的机身 Odometry 来源，缺省 /LIO/odom_vehicle。
    # 仿真通常传 --odom /quad_0/body_pose；输出仍固定为脚本旁 keypoint.yaml，不由该参数指定。
    parser.add_argument("--odom", default="/LIO/odom_vehicle", help="nav_msgs/Odometry topic to record.")
    return parser


# 先解析普通参数并初始化 ROS，再进入键盘录制循环；脚本退出不等于自动重启规划器。
def main():
    parser = build_arg_parser()
    args = parser.parse_args(strip_ros_args(sys.argv[1:]))

    rospy.init_node("keypoint_recorder")
    recorder = KeypointRecorder(args)
    recorder.run()


if __name__ == "__main__":
    main()
