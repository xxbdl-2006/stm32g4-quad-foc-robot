#!/usr/bin/env python3
"""
motor_tune.py —— 速度环阶跃辨识与增益建议。

流程（全自动，约 8 秒）：
  1. 让指定电机进入速度模式并使能；
  2. 依次施加 5 个速度阶跃（正负各若干），每个持续 800ms，记录实际速度；
  3. 对每个阶跃做一阶惯性+纯延迟拟合   G(s) = K/(τs+1) · e^(-Ls)；
  4. 用内模法（IMC）算 PI 初值：Kp = τ/(K·(λ+L)), Ki = Kp/τ，λ 取 τ/3；
  5. 打印建议值，问你是否写入（写 RAM，要持久化再调 SaveParams）。

用法：
    ros2 run 无关，直接 python3 tools/motor_tune.py --motor 0
    python3 tools/motor_tune.py --motor 2 --amp 3.0 --no-write

前置：hw_can 的 can_bridge 已在跑，且目标关节已解锁、负载已装上。
       （速度环辨识是对单关节做的，不需要起 arm_control —— 事实上
         辨识时更希望没有别的控制律在给同一个关节发指令。）
"""

import argparse
import math
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from hw_msgs.msg import MotorStateArray, MotorCommandArray
    from hw_msgs.srv import SetMotorGains, SaveParams
except ImportError:
    sys.exit("需要 ROS2 环境：先 source /opt/ros/humble/setup.bash 与 install/setup.bash")


class Tuner(Node):
    def __init__(self, motor_id, tol):
        super().__init__("motor_tune")
        self.motor_id = motor_id
        self.tol = tol
        self.samples = []          # (t, vel)
        self.latest = None
        self.t0 = None
        self.sub = self.create_subscription(
            MotorStateArray, "/hw/motor_states", self.on_state, 10)
        self.pub = self.create_publisher(MotorCommandArray, "/hw/motor_commands", 10)
        self.cli_gains = self.create_client(SetMotorGains, "/hw/set_motor_gains")
        self.cli_save = self.create_client(SaveParams, "/hw/save_params")

    def on_state(self, msg):
        if msg.motors is None or len(msg.motors) <= self.motor_id:
            return
        m = msg.motors[self.motor_id]
        self.latest = (m.position, m.velocity, m.fault)
        if self.t0 is not None:
            self.samples.append((time.monotonic() - self.t0, m.velocity))

    def command(self, setpoint, limit=4.0, enable=True, mode=2):
        arr = MotorCommandArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for i in range(4):
            from hw_msgs.msg import MotorCommand
            c = MotorCommand()
            c.motor_id = i
            c.mode = mode if i == self.motor_id else 0
            c.setpoint = float(setpoint) if i == self.motor_id else 0.0
            c.current_limit = float(limit)
            c.enable = enable and (i == self.motor_id)
            c.brake = False
            c.reset_fault = True
            arr.commands.append(c)
        self.pub.publish(arr)

    def spin_for(self, seconds, dt=0.02):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=dt)


def fit_first_order(ts, vs, target):
    """极简一阶拟合：直接用 63.2% 上升时间求 τ，稳态值求 K。"""
    if len(ts) < 10:
        return None
    steady = sum(vs[-20:]) / min(20, len(vs))
    if abs(steady) < 1e-3 or abs(target) < 1e-3:
        return None
    k = steady / target
    thresh = 0.632 * steady
    tau = None
    for t, v in zip(ts, vs):
        if (target > 0 and v >= thresh) or (target < 0 and v <= thresh):
            tau = t
            break
    if tau is None or tau <= 0.0:
        return None
    # 死区时间：第一次有明显响应（>2% 稳态）的时刻
    delay = 0.0
    for t, v in zip(ts, vs):
        if abs(v) > 0.02 * abs(steady):
            delay = t
            break
    return k, tau, delay, steady


def imc_pi(k, tau, delay, lam_factor=1.0 / 3.0):
    lam = max(tau * lam_factor, 1e-3)
    kp = tau / (k * (lam + delay))
    ki = kp / tau
    return kp, ki


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--motor", type=int, default=0, choices=[0, 1, 2, 3])
    ap.add_argument("--amp", type=float, default=2.0, help="阶跃幅值 rad/s")
    ap.add_argument("--no-write", action="store_true", help="只打印不写入")
    args = ap.parse_args()

    rclpy.init()
    node = Tuner(args.motor, 0.02)

    node.get_logger().info("等待 /hw/motor_states …")
    deadline = time.monotonic() + 5.0
    while node.latest is None and time.monotonic() < deadline:
        node.spin_for(0.1)
    if node.latest is None:
        node.get_logger().error("拿不到电机状态，检查 CAN 总线与驱动板供电")
        rclpy.shutdown()
        return 1

    if node.latest[2] != 0:
        node.get_logger().error(f"电机 {args.motor} 当前有故障 0x{node.latest[2]:04X}，"
                                f"先排除故障再做辨识")
        rclpy.shutdown()
        return 1

    results = []
    for step in (+args.amp, -args.amp, +args.amp * 0.5, -args.amp * 0.5):
        node.get_logger().info(f"阶跃 -> {step:+.2f} rad/s")
        node.samples.clear()

        # 先回零速稳定 300ms
        node.command(0.0)
        node.spin_for(0.3)

        node.t0 = time.monotonic()
        node.command(step)
        node.spin_for(0.8)
        node.t0 = None

        ts = [s[0] for s in node.samples]
        vs = [s[1] for s in node.samples]
        fit = fit_first_order(ts, vs, step)
        if fit is None:
            node.get_logger().warn("该阶跃未得到有效响应，跳过（可能是电流限幅或机械卡滞）")
            continue
        k, tau, delay, steady = fit
        node.get_logger().info(
            f"  K={k:.3f}  τ={tau*1000:.1f}ms  延迟={delay*1000:.1f}ms  "
            f"稳态={steady:+.3f} rad/s  跟随误差={(1-abs(steady/step))*100:.1f}%")
        results.append((k, tau, delay))

    node.command(0.0)
    node.spin_for(0.3)

    if not results:
        node.get_logger().error("没有任何有效阶跃结果")
        rclpy.shutdown()
        return 1

    # 取各次辨识的中位数，抑制单次异常
    ks = sorted(r[0] for r in results)
    taus = sorted(r[1] for r in results)
    delays = sorted(r[2] for r in results)
    mid = len(results) // 2
    k, tau, delay = ks[mid], taus[mid], delays[mid]

    kp, ki = imc_pi(k, tau, delay)
    # 保守系数：真机上按 0.7 倍写入，避免"理论最优在实物上振荡"
    kp_w, ki_w = kp * 0.7, ki * 0.7

    print("\n" + "=" * 62)
    print(f"  速度环辨识结果（电机 M{args.motor}）")
    print("=" * 62)
    print(f"  直流增益 K      : {k:.4f}  (rad/s) / (rad/s 指令)")
    print(f"  时间常数 τ      : {tau*1000:.1f} ms")
    print(f"  纯延迟 L        : {delay*1000:.1f} ms")
    print(f"  IMC 理论值      : Kp={kp:.4f}  Ki={ki:.4f}")
    print(f"  建议写入值(×0.7): Kp={kp_w:.4f}  Ki={ki_w:.4f}")
    print("=" * 62)
    print("  说明：K 明显小于 1 说明存在稳态误差，多半是电流限幅或摩擦；")
    print("        延迟 > 20ms 说明上位机侧有排队，检查 CPU 负载与 CAN 丢帧。")
    print()

    if args.no_write:
        node.get_logger().info("--no-write，未写入驱动板")
        rclpy.shutdown()
        return 0

    if not node.cli_gains.wait_for_service(timeout_sec=2.0):
        node.get_logger().error("/hw/set_motor_gains 服务不可用")
        rclpy.shutdown()
        return 1

    req = SetMotorGains.Request()
    req.motor_id = args.motor
    req.loop = 1                      # LOOP_VELOCITY
    req.kp, req.ki, req.kd = float(kp_w), float(ki_w), 0.0
    fut = node.cli_gains.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
    res = fut.result()
    if res is None or not res.success:
        node.get_logger().error(f"写入失败：{res.message if res else '超时'}")
        rclpy.shutdown()
        return 1
    node.get_logger().info(f"已写入，回读 Kp={res.kp_readback:.4f} Ki={res.ki_readback:.4f}")

    ans = input("写入驱动板 Flash 使其持久化？[y/N] ").strip().lower()
    if ans == "y" and node.cli_save.wait_for_service(timeout_sec=2.0):
        s = SaveParams.Request()
        s.motor_id = 0xFF
        f2 = node.cli_save.call_async(s)
        rclpy.spin_until_future_complete(node, f2, timeout_sec=3.0)
        r = f2.result()
        node.get_logger().info(f"保存：{r.message if r else '超时'}")

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
