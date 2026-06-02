Mission: The hood is responsible for moving to the commanded launch angle required for accurate and consistent scoring.


States: OFF, MANUAL_UP, MANUAL_DOWN, MOVING_TO_TARGET, AT_TARGET, FAULT


Events: RequestedManualUp, RequestedManualDown, RequestedTargetAngle, TargetAngleReached, TargetAngleLost, RequestedOff, LimitReached, MoveTimeout, FaultDetected, RequestedReset


Transitions: OFF+RequestedManualUp=MANUAL_UP, OFF+RequestedManualDown=MANUAL_DOWN, OFF+RequestedTargetAngle=MOVING_TO_TARGET, MANUAL_UP+RequestedOff=OFF, MANUAL_DOWN+RequestedOff=OFF, MANUAL_UP+RequestedTargetAngle=MOVING_TO_TARGET, MANUAL_DOWN+RequestedTargetAngle=MOVING_TO_TARGET, MOVING_TO_TARGET+TargetAngleReached=AT_TARGET, MOVING_TO_TARGET+RequestedOff=OFF, MOVING_TO_TARGET+LimitReached=FAULT, MOVING_TO_TARGET+MoveTimeout=FAULT, AT_TARGET+TargetAngleLost=MOVING_TO_TARGET, AT_TARGET+RequestedOff=OFF, AT_TARGET+RequestedTargetAngle=MOVING_TO_TARGET, FAULT+RequestedReset=OFF, ANY_STATE+FaultDetected=FAULT


Properties: targetAngleDeg, currentAngleDeg, hoodSpeed, angleErrorDeg, angleToleranceDeg, hoodVelocityDegPerSec, velocityToleranceDegPerSec, moveTimeoutSec, hasFault, faultReason

Derived Conditions: isAtTargetAngle, isMoving, atLimit, canAim


Faults: MOTOR_DISCONNECTED, ENCODER_DISCONNECTED, LIMIT_REACHED, MOVE_TIMEOUT, UNKNOWN


Requirements: The hood shall command negative hoodSpeed when MANUAL_UP.

The hood shall command positive hoodSpeed when MANUAL_DOWN.

The hood shall command targetAngleDeg when MOVING_TO_TARGET.

The hood shall enter AT_TARGET when currentAngleDeg is within angleToleranceDeg of targetAngleDeg.

The hood shall enter FAULT if LimitReached, MoveTimeout, or FaultDetected is received.

The hood shall stop motor output immediately when entering OFF or FAULT.

The hood shall log every state transition with timestamp, previous state, new state, and triggering event.


Acceptance Tests:
Requirement 1:
Trigger RequestedManualUp from OFF.
Verify state becomes MANUAL_UP.
Verify motor output is negative hoodSpeed.
Trigger RequestedOff.
Verify state becomes OFF.
Verify motor output is stopped.

Requirement 2:
Trigger RequestedManualDown from OFF.
Verify state becomes MANUAL_DOWN.
Verify motor output is positive hoodSpeed.
Trigger RequestedOff.
Verify state becomes OFF.
Verify motor output is stopped.

Requirement 3:
Set targetAngleDeg to 60.
Trigger RequestedTargetAngle from OFF.
Verify state becomes MOVING_TO_TARGET.
Verify currentAngleDeg reaches targetAngleDeg within angleToleranceDeg.
Verify state becomes AT_TARGET.
Verify canAim is true.

Requirement 4:
Set targetAngleDeg to 60.
Trigger RequestedTargetAngle from OFF.
Simulate currentAngleDeg staying outside angleToleranceDeg until moveTimeoutSec expires.
Verify state becomes FAULT.
Verify faultReason = MOVE_TIMEOUT.
Verify motor output is stopped.

Requirement 5:
Trigger OFF+RequestedTargetAngle=MOVING_TO_TARGET.
Inspect log output.
Verify log contains:
- timestamp
- previousState = OFF
- newState = MOVING_TO_TARGET
- event = RequestedTargetAngle
