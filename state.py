import time

KEYBOT_STATE_IDLE = 0
KEYBOT_PRESSING_LEFT = 1
KEYBOT_PRESSING_RIGHT = 2
KEYBOT_RETURNING_TO_CENTER_FROM_LEFT = 3
KEYBOT_RETURNING_TO_CENTER_FROM_RIGHT = 4
KEYBOT_ERROR_PRESSING_LEFT = 5
KEYBOT_ERROR_PRESSING_RIGHT = 6
KEYBOT_ERROR_RETURNING_TO_CENTER_FROM_LEFT = 7
KEYBOT_ERROR_RETURNING_TO_CENTER_FROM_RIGHT = 8
KEYBOT_STATE_EMERGENCY_RESET = 9

motorTimeout = 1000
returnTimeout = 1000

currentState = KEYBOT_STATE_IDLE
previousState = KEYBOT_STATE_IDLE

startTime = int(round(time.time() * 1000))

def update_state_machine():
    global currentState, previousState, startTime

    newState = currentState

    if newState != previousState:
        startTime = int(round(time.time() * 1000))

    elapsedTime = int(round(time.time() * 1000)) - startTime

    if currentState == KEYBOT_STATE_IDLE:
        pass

    elif currentState == KEYBOT_PRESSING_LEFT:
        # motor1Forward()2
        pass

    # Add other cases here

    if newState != previousState:
        print(f"State changed from {previousState} to {newState}")
        previousState = newState


# Simulate inputs here
currentState = KEYBOT_PRESSING_LEFT
update_state_machine()

