import openarm_driver
import time

cfgs = openarm_driver.available_configs()
print(cfgs)
# ['openarm_cell', 'openarm_cell_higher_pd', 'openarm_pedestal']

config = openarm_driver.Config("openarm_pedestal")
arm = openarm_driver.SingleArmDriver("right_arm", config) # switch left_arm or right_arm

# Or make it the default for every driver created afterwards.
openarm_driver.set_default_config(config)

try:
    print("start")
    start_time = time.time()
    arm.start()
    start_end_time = time.time()
    print(f"time taken to start in ms: {(start_end_time - start_time) * 1000}")
    print("started")
    time.sleep(1)
    print("waited 1 second")

    fetch_start_time = time.time()
    cur_position = arm.fetch_position()
    fetch_end_time = time.time()
    print(f"time taken to fetch position in ms: {(fetch_end_time - fetch_start_time) * 1000}")
    print(f"current position: {cur_position}")


    next_postion = cur_position + 0.1
    print(f"next position: {next_postion}")


    # arm.smooth_move(next_postion, hz=50, duration=1)
    # smooth_time = time.time()
    # print(f"time taken to smooth move in ms: {(smooth_time - start_time) * 1000}")
finally:
    arm.stop()