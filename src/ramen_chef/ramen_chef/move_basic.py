import time
import rclpy
import DR_init
from std_msgs.msg import Int32, Bool

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight"
ROBOT_TOOL = "GripperDA_v1"
VEL = 60
ACC = 60

current_progress = 0
stop_requested = False
recovery_requested = False
end_received = False
progress_pub = None

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# Publishes current robot task status to a topic
def publish_state(num: int) -> None:
    """Publish progress state to /progress_state. Keeps track of last progress (except 7)."""
    global progress_pub, current_progress

    if progress_pub is None:
        print("progress_pub is not initialized")
        return

    msg = Int32()
    msg.data = int(num)
    progress_pub.publish(msg)

    if num != 7:
        current_progress = num

def initialize_robot() -> None:
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TCP)
    set_tcp(ROBOT_TOOL)

# Open gripper
def open_gripper(wait: float = 1.0) -> None:
    from DSR_ROBOT2 import set_digital_output
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    time.sleep(wait)

# Close gripper
def close_gripper(wait: float = 1.0) -> None:
    from DSR_ROBOT2 import set_digital_output
    set_digital_output(1, 1)
    set_digital_output(2, 1)
    time.sleep(wait)

# Close gripper gently to prevent excessive crushing of the water bottle
def water_close_gripper(wait: float = 1.0) -> None:
    from DSR_ROBOT2 import set_digital_output
    set_digital_output(1, 1)
    set_digital_output(2, 0)
    time.sleep(wait)

# Move robot to home position
def home_return(vel: int = VEL, acc: int = ACC) -> None:
    from DSR_ROBOT2 import movej
    movej([0, 0, 90, 0, 90, 0], vel=vel, acc=acc)

def end_motion() -> None:
    from DSR_ROBOT2 import movej
    move1 = [2.68, -32.42, 124.53, 35.66, 59.13, 0.00]
    move2 = [6.10, 23.80, 70.12, 31.39, 109.08, 0.00]
    movej(move1, 60, 60)
    close_gripper()
    open_gripper()
    movej(move2, 60, 60)
    close_gripper()
    open_gripper()

# Perform recovery motion in case of an error
def recovery_motion() -> None:
    from DSR_ROBOT2 import movel, movej, posx, DR_BASE

    print(f"Recovery motion start (progress={current_progress})")
    open_gripper()

    if current_progress in (1, 2, 8):
        L_pot_release_up = posx([340.00, 275.58, 454.66, 48.22, 177.91, -131.07])
        L_pot_grip = posx([408.41, -222.64, 481.84, 58.44, -179.90, -32.30])

        movel(L_pot_release_up, vel=VEL, acc=ACC)
        movel(posx([0, 0, -101, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)
        close_gripper()

        movel(posx([0, 0, 100, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)
        movel(L_pot_grip, vel=VEL, acc=ACC)
        movel(posx([0, 0, -182.6, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)

        open_gripper()
        print("Pot returned to original position")
        home_return()
    else:
        L_pot_grip = posx([340.00, 275.58, 454.66, 48.25, 177.91, -39.92])
        J_pot_move = [-7.28, 16.79, 80.92, -1.40, 82.37, 83.53]

        movel(L_pot_grip, vel=VEL, acc=ACC)
        movej([0, 0, 0, 0, 0, -90], vel=VEL, acc=ACC, mod=1)

        movel(posx([0, 0, -101, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)
        close_gripper()
        movel(posx([0, 0, 80, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)

        home_return()
        movej([0, 0, 0, 0, 0, 90], vel=VEL, acc=ACC, mod=1)
        movej(J_pot_move, vel=VEL, acc=ACC)

        movel(posx([134, 0, 0, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)
        movel(posx([0, 0, -50, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)

        open_gripper()
        print("Pot discarded")
        home_return()

    print("Recovery motion done")

# Main function for the entire task sequence
def perform_task(node, mode: int) -> None:
    from DSR_ROBOT2 import movej, movel, posx, DR_BASE, get_digital_input

    global progress_pub

    if progress_pub is None:
        progress_pub = node.create_publisher(Int32, "/progress_state", 10)

    def check_end() -> None:
        if end_received:
            home_return(vel=50, acc=50)
            raise Exception("END")

    def check_stop() -> None:
        if stop_requested:
            home_return(vel=50, acc=50)
            raise Exception("STOP")

    def notify_material_empty(step_name: str) -> None:
        publish_state(7)
        print(f"No material: {step_name}")
        raise Exception("NO_MATERIAL")

    def check_grip(step_name: str) -> None:
        di = get_digital_input(1)
        if di == 0:
            notify_material_empty(step_name)

    def move_pot() -> None:
        publish_state(1)
        check_stop()
        J_pot_grip = [-30.86, 16.01, 91.07, -0.22, 72.95, -30.77]
        L_pot_release_up = posx([340.00, 275.58, 455.66, 48.25, 177.91, -39.92])

        home_return()
        open_gripper()

        movej(J_pot_grip, vel=VEL, acc=ACC)
        close_gripper()
        check_grip("pick pot")

        movel(L_pot_release_up, vel=VEL, acc=ACC)
        movel(posx([0, 0, -101, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)
        open_gripper()
        movel(posx([0, 0, 100, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)
        home_return()

    def drop_water() -> None:
        check_stop()
        publish_state(2)

        L_water_grip = posx([486.43, 134.10, 318.03, 10.36, 145.90, -83.99])
        L_water_out = posx([396.91, 118.61, 450.07, 9.98, 145.85, -84.27])
        L_water_drop = posx([134.57, 136.44, 430.30, 24.24, 158.55, -94.97])
        L_water_dropping_first = posx([208.78, 164.55, 412.99, 152.18, -173.66, 37.15])
        L_water_dropping_second = posx([281.85, 238.05, 411.82, 53.92, -151.42, -62.09])
        L_water_dropping_third = posx([350.03, 308.92, 335.97, 50.91, -120.55, -63.43])
        L_water_waste = posx([633.45, -67.80, 331.46, 11.74, 178.92, -79.46])

        water_velacc = 10

        open_gripper()
        movel(L_water_out, vel=VEL, acc=ACC)
        movel(L_water_grip, vel=VEL, acc=ACC)
        water_close_gripper()
        check_grip("pick water bottle")

        movel(L_water_out, vel=VEL, acc=ACC)
        movel(L_water_drop, vel=VEL, acc=ACC)

        movel(L_water_dropping_first, vel=water_velacc, acc=water_velacc)
        time.sleep(2)
        movel(L_water_dropping_second, vel=water_velacc, acc=water_velacc)
        time.sleep(2)
        movel(L_water_dropping_third, vel=water_velacc, acc=water_velacc)
        time.sleep(2)

        home_return()

        movel(L_water_waste, vel=VEL, acc=ACC)
        movel(posx([0, 0, -50, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)
        open_gripper()
        home_return()

    def take_noodle() -> None:
        publish_state(3)
        check_stop()
        L_noodle_case = posx([496.50, 286.33, 319.35, 5.35, 153.96, -90.67])
        L_noodle_out = posx([427.51, 281.88, 460.77, 4.86, 153.91, -91.04])
        L_noodle_upper_pot = posx([336.72, 57.84, 522.26, 74.88, 158.15, 159.68])
        L_noodle_down_pot = posx([342.26, 119.16, 360.56, 75.13, 158.31, 159.81])

        movel(L_noodle_case, vel=VEL, acc=ACC)
        close_gripper()
        check_grip("pick noodle")

        movel(L_noodle_out, vel=VEL, acc=ACC)
        movel(L_noodle_upper_pot, vel=VEL, acc=ACC)
        movel(L_noodle_down_pot, vel=VEL, acc=ACC)

        open_gripper()
        movel(posx([0, 0, 50, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)
        home_return()

    def dance(n: int) -> None:
        J1 = [0.00, 16.11, 81.19, 12.54, -61.66, 0.00]
        J2 = [0.00, -16.87, 107.71, 12.77, -125.63, 0.00]
        open_gripper()

        for _ in range(int(n)):
            check_stop()
            check_end()
            movej(J1, vel=60, acc=60)
            movej(J2, vel=60, acc=60)

    # Action of pouring soup/toppings using a cup
    def pour_with_cup(cup_grip, cup_case, cup_back, step_name: str) -> None:
        J_sauce_upper_pot = [39.23, -36.98, 125.59, -10.72, 49.46, -83.16]
        J_sauce_pour = [43.79, 26.84, 73.89, -21.61, 114.97, -111.39]

        VELACC_01 = 40
        VELACC_02 = 30

        movej(cup_grip, vel=VEL, acc=ACC)
        movel(cup_case, vel=VEL, acc=ACC)
        close_gripper()
        check_grip(step_name)
        movel(cup_back, vel=VEL, acc=ACC)

        movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
        time.sleep(0.5)
        movej(J_sauce_pour, vel=VELACC_02, acc=VELACC_02)

        movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
        time.sleep(0.5)

        movej(cup_grip, vel=VEL, acc=ACC)
        movel(cup_case, vel=VEL, acc=ACC)
        open_gripper()
        movel(cup_back, vel=VEL, acc=ACC)
        home_return()

    # Execute soup and topping pouring sequence based on the selected mode
    def pour_cup() -> None:
        publish_state(4)
        check_stop()

        '''
        cup1 cup2 cup3

        robotarm
        '''
        J_cup1_grip = [-1.91, 9.67, 116.94, -14.32, -29.80, -73.38]
        L_cup1_case = posx([613.63, 18.73, 285.86, 5.94, 97.63, -84.76])
        L_cup1_back = posx([382.21, 7.73, 303.07, 5.58, 97.51, -84.87])

        J_cup2_grip = [-17.15, 8.15, 118.16, -37.14, -34.86, -56.71]
        L_cup2_case = posx([605.37, -83.77, 292.60, 4.50, 97.19, -85.68])
        L_cup2_back = posx([359.70, -89.65, 307.57, 3.47, 96.96, -86.01])

        J_cup3_grip = [-39.15, 4.52, 121.18, -62.17, -47.37, -40.28]
        L_cup3_case = posx([601.76, -190.73, 292.83, 3.14, 96.89, -86.31])
        L_cup3_back = posx([340.48, -198.37, 314.60, 2.14, 96.65, -86.58])

        plan = {
            0: [(J_cup1_grip, L_cup1_case, L_cup1_back, "pick sauce cup1 (mode 0)")],
            1: [
                (J_cup1_grip, L_cup1_case, L_cup1_back, "pick sauce cup1 (mode 1)"),
                (J_cup2_grip, L_cup2_case, L_cup2_back, "pick sauce cup2 (mode 1)"),
            ],
            2: [
                (J_cup1_grip, L_cup1_case, L_cup1_back, "pick sauce cup1 (mode 2)"),
                (J_cup3_grip, L_cup3_case, L_cup3_back, "pick sauce cup3 (mode 2)"),
            ],
            3: [
                (J_cup1_grip, L_cup1_case, L_cup1_back, "pick sauce cup1 (mode 3)"),
                (J_cup2_grip, L_cup2_case, L_cup2_back, "pick sauce cup2 (mode 3)"),
                (J_cup3_grip, L_cup3_case, L_cup3_back, "pick sauce cup3 (mode 3)"),
            ],
        }

        for cup_grip, cup_case, cup_back, step_name in plan.get(int(mode), []):
            pour_with_cup(cup_grip, cup_case, cup_back, step_name)

        move_pot()
        drop_water()

        publish_state(6)
        dance(7)
        home_return()

        pour_cup()
        home_return()

        open_gripper()
        take_noodle()

        publish_state(5)

        print("Cooking done. Waiting for END signal...")
        while not end_received:
            rclpy.spin_once(node, timeout_sec=0.2)
            dance(1)

        publish_state(8)
        end_motion()
        publish_state(0)
        home_return()


# ROS node initialization and execution
def main(args=None) -> None:
    global stop_requested, recovery_requested, progress_pub, end_received

    rclpy.init()
    node = rclpy.create_node("pot_robot", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    progress_pub = node.create_publisher(Int32, "/progress_state", 10)

    start_received = False
    mode = None

    def start_cb(msg):
        nonlocal start_received
        if msg.data:
            start_received = True

    def mode_cb(msg):
        nonlocal mode
        mode = msg.data

    def stop_cb(msg):
        global stop_requested
        if msg.data:
            stop_requested = True

    def recovery_cb(msg):
        global recovery_requested
        if msg.data:
            recovery_requested = True

    def end_cb(msg):
        global end_received
        if msg.data:
            end_received = True

    node.create_subscription(Bool, "/start_signal", start_cb, 10)
    node.create_subscription(Int32, "/mode_select", mode_cb, 10)
    node.create_subscription(Bool, "/stop_signal", stop_cb, 10)
    node.create_subscription(Bool, "/recovery_signal", recovery_cb, 10)
    node.create_subscription(Bool, "/end_signal", end_cb, 10)

    print("Robot ready")

    while rclpy.ok():
        start_received = False
        mode = None
        stop_requested = False
        recovery_requested = False
        end_received = False

        while rclpy.ok() and not start_received:
            rclpy.spin_once(node, timeout_sec=0.2)
            if stop_requested:
                break

        if stop_requested:
            from DSR_ROBOT2 import movej
            try:
                movej([0, 0, 90, 0, 90, 0], vel=50, acc=50)
            except Exception:
                pass

            while not recovery_requested:
                rclpy.spin_once(node, timeout_sec=0.2)

            recovery_motion()
            publish_state(0)

            stop_requested = False
            recovery_requested = False
            continue

        while mode is None:
            rclpy.spin_once(node, timeout_sec=0.2)
            if stop_requested:
                break

        if stop_requested:
            continue

        try:
            initialize_robot()
            perform_task(node, int(mode))
        except Exception as e:
            from DSR_ROBOT2 import movej

            if str(e) in ("STOP", "NO_MATERIAL"):
                try:
                    movej([0, 0, 90, 0, 90, 0], vel=50, acc=50)
                except Exception:
                    pass

                while not recovery_requested:
                    rclpy.spin_once(node, timeout_sec=0.2)

                recovery_motion()
                publish_state(0)

                stop_requested = False
                recovery_requested = False
                continue

            if str(e) == "END":
                try:
                    movej([0, 0, 90, 0, 90, 0], vel=50, acc=50)
                except Exception:
                    pass
                recovery_motion()
                publish_state(0)
                continue

    rclpy.shutdown()

if __name__ == "__main__":
    main()
