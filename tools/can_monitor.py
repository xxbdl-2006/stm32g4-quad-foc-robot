#!/usr/bin/env python3
"""
can_monitor.py —— 在终端里把驱动板的自定义协议帧翻译成人话。

用法：
    python3 tools/can_monitor.py                    # 默认 can0
    python3 tools/can_monitor.py -i can1 -v         # 换接口 + 打印原始字节
    python3 tools/can_monitor.py --only motor       # 只看电机反馈
    python3 tools/can_monitor.py --csv out.csv      # 顺手落盘成 CSV

为什么要有这个东西：Wireshark 能看 CAN，但看到的是
    181#3A2F0000C8001E00
而它是
    M0  位置  12.09 rad  速度  1.200 rad/s  电流  0.030 A
排查问题时这两者的效率差一个数量级。

依赖：python-can（pip install python-can）
"""

import argparse
import struct
import sys
import time

try:
    import can
except ImportError:
    sys.exit("缺少 python-can： pip install python-can")

# ---- 与 firmware/Core/Inc/can_proto.h 保持一致 ----
NODE_NAMES = {0: "HOST", 1: "MAIN", 2: "EXP"}

CLS_HEARTBEAT = 0x0
CLS_FAST = 0x1
CLS_MOTION = 0x2
CLS_PARAM = 0x3
CLS_IO = 0x4
CLS_FLASH = 0x5
CLS_DIAG = 0x6
CLS_ESTOP = 0x7

CLS_NAMES = {
    CLS_HEARTBEAT: "HB", CLS_FAST: "FAST", CLS_MOTION: "MOTION",
    CLS_PARAM: "PARAM", CLS_IO: "IO", CLS_FLASH: "FLASH",
    CLS_DIAG: "DIAG", CLS_ESTOP: "ESTOP",
}

MOTOR_MODES = ["IDLE", "CURRENT", "VELOCITY", "POSITION",
               "OPENLOOP", "CALIBRATE", "BRAKE"]

FAULT_BITS = [
    "UNDERVOLT", "OVERVOLT", "OVERCURR", "MCU_HOT", "MOS_HOT",
    "ENC_SPI", "ENC_JUMP", "DRV_NFAULT", "CAN_BUSOFF", "CAN_TIMEOUT",
    "WATCHDOG", "CTRL_LATE", "PUMP_STALL", "CAM_SYNC_LOST",
]


def id_node(can_id):
    return (can_id >> 8) & 0x7


def id_class(can_id):
    return (can_id >> 4) & 0xF


def id_idx(can_id):
    return can_id & 0xF


def fault_str(flags):
    if flags == 0:
        return "-"
    names = [n for i, n in enumerate(FAULT_BITS) if flags & (1 << i)]
    return "|".join(names) if names else f"0x{flags:04X}"


def decode_motor_feedback(data):
    pos_mrad, vel_mrads, cur_ma = struct.unpack("<ihh", data[:8])
    return (pos_mrad * 1e-3, vel_mrads * 1e-3, cur_ma * 1e-3)


def decode_board_status(data):
    vbus_mv, temp_c10, power_w10, fl, st = struct.unpack("<HhhBB", data[:8])
    return (vbus_mv * 1e-3, temp_c10 * 0.1, power_w10 * 0.1, fl, st)


def decode_heartbeat(data):
    fw, st, fi, fl, en, up = struct.unpack("<HBBHBB", data[:8])
    return f"固件 {fw >> 12}.{(fw >> 8) & 0xF}.{fw & 0xFF} " \
           f"状态={st} 故障=0x{fl:04X}({fault_str(fl)}) " \
           f"使能=0b{en:04b} 运行={up}s"


def decode_motion(data):
    mode, setpoint, limit_ma, flags = struct.unpack("<BfHB", data[:8])
    mode_s = MOTOR_MODES[mode] if mode < len(MOTOR_MODES) else f"?{mode}"
    fl = []
    if flags & 0x1: fl.append("EN")
    if flags & 0x2: fl.append("BRAKE")
    if flags & 0x4: fl.append("RST")
    if flags & 0x8: fl.append("ESTOP")
    return f"{mode_s:9s} 设定={setpoint:9.4f} 限流={limit_ma/1000.0:5.2f}A [{'|'.join(fl)}]"


def decode_param(data):
    pid, val, op, res = struct.unpack("<HfBB", data[:8])
    ops = {0: "READ_REQ", 1: "WRITE", 2: "WRITE_ACK", 3: "READ_RESP", 4: "ERR"}
    if pid & 0x8000:
        return f"事件 code={pid & 0x7FFF} 载荷={val:.4f} motor={res}"
    return f"param=0x{pid:04X} 值={val:.5g} op={ops.get(op, op)}"


def decode_pump_state(data):
    st, duty, kpa10, flt, _ = struct.unpack("<BHHBB", data[:8])
    states = ["IDLE", "RAMP", "RUN", "FAULT", "SOFTSTOP"]
    return f"泵 {states[st] if st < 5 else st:8s} 占空={duty/10.0:5.1f}% " \
           f"压力={kpa10/10.0:5.1f}kPa 故障=0x{flt:02X}"


def decode_pump_cmd(data):
    cmd, duty, mode, tgt, _ = struct.unpack("<BHBHB", data[:8])
    cmds = {0: "STOP", 1: "RUN", 2: "SOFT_STOP"}
    modes = {0: "开环", 1: "压力环"}
    return f"泵指令 {cmds.get(cmd, cmd):10s} {modes.get(mode, mode)} " \
           f"占空={duty/10.0:.1f}% 目标={tgt/10.0:.1f}kPa"


def decode_cam_sync(data):
    fid, dt_us, dropped = struct.unpack("<IHH", data[:8])
    return f"相机同步 帧号={fid} 间隔={dt_us/1000.0:.2f}ms 丢帧={dropped}"


def decode_cam_latch(data):
    pos_mrad, vel_mrads, fid_lo = struct.unpack("<ihH", data[:8])
    return f"锁存 位置={pos_mrad*1e-3:8.4f}rad 速度={vel_mrads*1e-3:7.3f}rad/s " \
           f"帧号低16位={fid_lo}"


def decode_cam_cfg(data):
    en, rate, pulse, mode = struct.unpack("<BHHB", data[:8])
    return f"相机配置 使能={en} 频率={rate}Hz 脉宽={pulse}us 模式={mode}"


def render(msg, verbose=False):
    cid = msg.arbitration_id
    node = id_node(cid)
    cls = id_class(cid)
    idx = id_idx(cid)
    data = bytes(msg.data)
    t = time.strftime("%H:%M:%S", time.localtime()) + f".{msg.timestamp % 1:.3f}"[2:]

    prefix = f"{t} {NODE_NAMES.get(node, node):4s} {CLS_NAMES.get(cls, cls):6s} idx={idx:2d}"

    detail = None
    if cls == CLS_FAST:
        if idx < 4:
            pos, vel, cur = decode_motor_feedback(data)
            detail = f"M{idx} 位置={pos:9.4f}rad 速度={vel:7.3f}rad/s 电流={cur:6.3f}A"
        elif idx == 4:
            vbus, temp, power, fl, st = decode_board_status(data)
            detail = f"板卡 母线={vbus:5.2f}V MCU={temp:5.1f}C 功率={power:6.2f}W " \
                     f"状态={st} 故障=0x{fl:02X}"
    elif cls == CLS_HEARTBEAT:
        detail = decode_heartbeat(data)
    elif cls == CLS_MOTION:
        detail = f"M{idx} 指令 {decode_motion(data)}"
    elif cls == CLS_PARAM:
        detail = f"M{idx} {decode_param(data)}"
    elif cls == CLS_IO:
        if idx == 0:
            detail = decode_pump_cmd(data)
        elif idx == 1:
            detail = decode_pump_state(data)
        elif idx == 2:
            detail = decode_cam_cfg(data)
        elif idx == 3:
            detail = decode_cam_sync(data)
        elif 5 <= idx <= 8:
            detail = f"M{idx-5} {decode_cam_latch(data)}"
    elif cls == CLS_ESTOP:
        detail = "★ 急停广播 ★"

    if detail is None:
        detail = " ".join(f"{b:02X}" for b in data)

    line = f"{prefix}  {detail}"
    if verbose:
        line += f"    [{cid:03X}] {' '.join(f'{b:02X}' for b in data)}"
    return line


def main():
    ap = argparse.ArgumentParser(description="HWB-MC4G4 CAN 协议解析器")
    ap.add_argument("-i", "--interface", default="can0")
    ap.add_argument("-b", "--bitrate", type=int, default=1000000)
    ap.add_argument("-v", "--verbose", action="store_true", help="附加原始字节")
    ap.add_argument("--only", choices=["motor", "board", "pump", "cam", "param", "all"],
                    default="all")
    ap.add_argument("--csv", help="把 (时间, 电机, 位置, 速度, 电流) 落盘")
    ap.add_argument("--quiet-hb", action="store_true", help="隐藏心跳帧")
    args = ap.parse_args()

    bus = can.interface.Bus(channel=args.interface, bustype="socketcan")

    csv_f = None
    if args.csv:
        csv_f = open(args.csv, "w", encoding="utf-8")
        csv_f.write("t,motor,position_rad,velocity_rad_s,current_a\n")

    print(f"监听 {args.interface} @ {args.bitrate} bps，Ctrl-C 退出\n")
    try:
        for msg in bus:
            cid = msg.arbitration_id
            cls = id_class(cid)
            idx = id_idx(cid)

            if idx == 0 and id_node(cid) == 0:
                continue                       # 忽略自己发的
            if args.quiet_hb and cls == CLS_HEARTBEAT:
                continue
            if args.only == "motor" and not (cls == CLS_FAST and idx < 4):
                continue
            if args.only == "board" and not (cls == CLS_FAST and idx == 4):
                continue
            if args.only == "pump" and not (cls == CLS_IO and idx in (0, 1)):
                continue
            if args.only == "cam" and not (cls == CLS_IO and 2 <= idx <= 8):
                continue
            if args.only == "param" and cls != CLS_PARAM:
                continue

            print(render(msg, args.verbose))

            if csv_f is not None and cls == CLS_FAST and idx < 4:
                pos, vel, cur = decode_motor_feedback(bytes(msg.data))
                csv_f.write(f"{msg.timestamp:.6f},{idx},{pos:.6f},{vel:.6f},{cur:.6f}\n")
                csv_f.flush()

    except KeyboardInterrupt:
        print("\n已停止")
    finally:
        if csv_f:
            csv_f.close()
        bus.shutdown()


if __name__ == "__main__":
    main()
