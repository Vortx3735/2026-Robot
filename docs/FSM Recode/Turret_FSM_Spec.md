Mission: The turret is responsible for rotating the shooter to the commanded target position required for aiming.


States: OFF, MANUAL_LEFT, MANUAL_RIGHT, MOVING_TO_TARGET, AT_TARGET, FAULT


Events: RequestedManualLeft, RequestedManualRight, RequestedTargetPosition, TargetPositionReached, TargetPositionLost, RequestedOff, LimitReached, MoveTimeout, FaultDetected, RequestedReset


Transitions: OFF+RequestedManualLeft=MANUAL_LEFT, OFF+RequestedManualRight=MANUAL_RIGHT, OFF+RequestedTargetPosition=MOVING_TO_TARGET, MANUAL_LEFT+RequestedOff=OFF, MANUAL_RIGHT+RequestedOff=OFF, MANUAL_LEFT+RequestedTargetPosition=MOVING_TO_TARGET, MANUAL_RIGHT+RequestedTargetPosition=MOVING_TO_TARGET, MOVING_TO_TARGET+TargetPositionReached=AT_TARGET, MOVING_TO_TARGET+RequestedOff=OFF, MOVING_TO_TARGET+LimitReached=FAULT, MOVING_TO_TARGET+MoveTimeout=FAULT, AT_TARGET+TargetPositionLost=MOVING_TO_TARGET, AT_TARGET+RequestedOff=OFF, AT_TARGET+RequestedTargetPosition=MOVING_TO_TARGET, FAULT+RequestedReset=OFF, ANY_STATE+FaultDetected=FAULT


Properties: targetPositionRotations, currentPositionRotations, turretSpeed, positionErrorRotations, positionToleranceRotations, currentRPS, velocityToleranceRPS, moveTimeoutSec, hasFault, faultReason

Derived Conditions: isAtTargetPosition, isMoving, atLimit, canAim


Faults: MOTOR_DISCONNECTED, ENCODER_DISCONNECTED, LIMIT_REACHED, MOVE_TIMEOUT, UNKNOWN


Requirements: The turret shall command negative turretSpeed when MANUAL_LEFT.

The turret shall command positive turretSpeed when MANUAL_RIGHT.

The turret shall command targetPositionRotations when MOVING_TO_TARGET.

The turret shall enter AT_TARGET when currentPositionRotations is within positionToleranceRotations of targetPositionRotations.

The turret shall enter FAULT if LimitReached, MoveTimeout, or FaultDetected is received.

The turret shall stop motor output immediately when entering OFF or FAULT.

The turret shall log every state transition with timestamp, previous state, new state, and triggering event.


Acceptance Tests:
Requirement 1:
Trigger RequestedManualLeft from OFF.
Verify state becomes MANUAL_LEFT.
Verify motor output is negative turretSpeed.
Trigger RequestedOff.
Verify state becomes OFF.
Verify motor output is stopped.

Requirement 2:
Trigger RequestedManualRight from OFF.
Verify state becomes MANUAL_RIGHT.
Verify motor output is positive turretSpeed.
Trigger RequestedOff.
Verify state becomes OFF.
Verify motor output is stopped.

Requirement 3:
Set targetPositionRotations to 0.25.
Trigger RequestedTargetPosition from OFF.
Verify state becomes MOVING_TO_TARGET.
Verify currentPositionRotations reaches targetPositionRotations within positionToleranceRotations.
Verify state becomes AT_TARGET.
Verify canAim is true.

Requirement 4:
Set targetPositionRotations to 0.25.
Trigger RequestedTargetPosition from OFF.
Simulate currentPositionRotations staying outside positionToleranceRotations until moveTimeoutSec expires.
Verify state becomes FAULT.
Verify faultReason = MOVE_TIMEOUT.
Verify motor output is stopped.

Requirement 5:
Trigger OFF+RequestedTargetPosition=MOVING_TO_TARGET.
Inspect log output.
Verify log contains:
- timestamp
- previousState = OFF
- newState = MOVING_TO_TARGET
- event = RequestedTargetPosition
