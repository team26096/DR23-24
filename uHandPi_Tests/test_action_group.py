import sys
import LeArm
import time

if __name__ == "__main__":
    LeArm.initLeArm([0, 0, 0, 0, 0, 0])
    times = 1 if len(sys.argv) < 2 else int(sys.argv[2])
    for _ in range(0, times):
        LeArm.runActionGroup(sys.argv[1], 1)
        time.sleep(2)
