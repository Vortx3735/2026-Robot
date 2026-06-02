Mission: The hopper is responsible for moving fuel from the intake path toward the tunnel and shooter.


States: OFF, INTAKING, OUTTAKING


Events: RequestedIntake, RequestedOuttake, RequestedOff, JamDetected, JamClearCompleted, FaultDetected


Transitions: OFF+RequestedIntake=INTAKING, OFF+RequestedOuttake=OUTTAKING, INTAKING+RequestedOff=OFF, INTAKING+RequestedOuttake=OUTTAKING, INTAKING+JamDetected=OUTTAKING, OUTTAKING+JamClearCompleted=INTAKING, OUTTAKING+RequestedOff=OFF, OUTTAKING+RequestedIntake=INTAKING, ANY_STATE+FaultDetected=OFF


Properties: hopperSpeed, slowHopperSpeed, currentRPS, statorCurrent, currentLimit, hasFault, faultReason

Derived Conditions: isRunning, isIntaking, isOuttaking, isJammed


Faults: MOTOR_DISCONNECTED, OVERCURRENT, JAMMED, UNKNOWN


Requirements: The hopper shall command positive hopperSpeed when INTAKING.

The hopper shall command negative hopperSpeed when OUTTAKING.

The hopper shall use slowHopperSpeed when the shooter command requests slow feeding.

The hopper shall briefly enter OUTTAKING if JamDetected is received while INTAKING.

The hopper shall stop motor output when OFF.

The hopper shall enter OFF if FaultDetected is received.

The hopper shall log every state transition with timestamp, previous state, new state, and triggering event.


Acceptance Tests:
Requirement 1:
Set hopperSpeed to 0.3.
Trigger RequestedIntake from OFF.
Verify state becomes INTAKING.
Verify motor output is positive hopperSpeed.

Requirement 2:
Set hopperSpeed to 0.3.
Trigger RequestedOuttake from OFF.
Verify state becomes OUTTAKING.
Verify motor output is negative hopperSpeed.

Requirement 3:
Start in INTAKING.
Simulate statorCurrent greater than or equal to currentLimit.
Verify JamDetected is emitted.
Verify state becomes OUTTAKING.
Verify motor output is negative hopperSpeed.
Trigger JamClearCompleted.
Verify state becomes INTAKING.

Requirement 4:
Trigger RequestedOff from INTAKING.
Verify state becomes OFF.
Verify motor output is stopped.

Requirement 5:
Trigger OFF+RequestedIntake=INTAKING.
Inspect log output.
Verify log contains:
- timestamp
- previousState = OFF
- newState = INTAKING
- event = RequestedIntake
