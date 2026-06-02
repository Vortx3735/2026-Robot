Mission: The intake is responsible for moving fuel from outside the robot into the robot.


States: OFF, INTAKING, OUTTAKING


Events: RequestedIntake, RequestedOuttake, RequestedOff, FaultDetected


Transitions: OFF+RequestedIntake=INTAKING, OFF+RequestedOuttake=OUTTAKING, INTAKING+RequestedOff=OFF, INTAKING+RequestedOuttake=OUTTAKING, OUTTAKING+RequestedOff=OFF, OUTTAKING+RequestedIntake=INTAKING, ANY_STATE+FaultDetected=OFF


Properties: intakeSpeed, currentRPS, hasFault, faultReason

Derived Conditions: isRunning, isIntaking, isOuttaking


Faults: MOTOR_DISCONNECTED, OVERCURRENT, UNKNOWN


Requirements: The intake shall command positive intakeSpeed when INTAKING.

The intake shall command negative intakeSpeed when OUTTAKING.

The intake shall stop motor output when OFF.

The intake shall enter OFF if FaultDetected is received.

The intake shall log every state transition with timestamp, previous state, new state, and triggering event.


Acceptance Tests:
Requirement 1:
Set intakeSpeed to 1.0.
Trigger RequestedIntake from OFF.
Verify state becomes INTAKING.
Verify motor output is positive intakeSpeed.

Requirement 2:
Set intakeSpeed to 1.0.
Trigger RequestedOuttake from OFF.
Verify state becomes OUTTAKING.
Verify motor output is negative intakeSpeed.

Requirement 3:
Trigger RequestedOff from INTAKING.
Verify state becomes OFF.
Verify motor output is stopped.

Requirement 4:
Trigger OFF+RequestedIntake=INTAKING.
Inspect log output.
Verify log contains:
- timestamp
- previousState = OFF
- newState = INTAKING
- event = RequestedIntake
