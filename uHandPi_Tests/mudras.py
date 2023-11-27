import LeArm
import time

MUDRA_LIST = ["Ardhapathaka", "Tripathaka", "Chandrakala", "Sarpasirsha"]

def get_mudra_to_perform():
    print("Enter the mudra to perform:")
    for i in range(len(MUDRA_LIST)):
        print(str(i) + " - " + MUDRA_LIST[i])
    mudra_index = int(input("Enter the index of the mudra to perform: "))
    print("Performing mudra: " + MUDRA_LIST[mudra_index])
    return mudra_index


if __name__ == "__main__":
    LeArm.initLeArm([0, 0, 0, 0, 0, 0])
    LeArm.runActionGroup("mudra_" + str(mudra_index), 1)


    mudra_index = get_mudra_to_perform()
    LeArm.runActionGroup("mudra_" + str(mudra_index), 1)

# 4 Fingers
    # 500 - Fully Folded
    # 2500 - Fully Open pointed upwards
    # Servo 2 is the nearest to the Thumb (Index Finger)
    # Servo 3
    # Servo 4
    # Servo 5

# Thumb - Servo 1
    # 500 - Stretched out
    # 2500 - Fully Folded in
# Camera - Servo 6
    # 500 - Turned Left by 90 deg
    # 1500 - Face front
    # 2500 - Turned Right by 90 deg
