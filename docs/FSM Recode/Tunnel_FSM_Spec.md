Mission: The tunnel is responsible for moving fuel between the hopper and shooter using the top and bottom tunnel motors.


States: OFF, INTAKING, OUTTAKING


Events: RequestedIntake, RequestedOuttake, RequestedOff, FaultDetected


Transitions: OFF+RequestedIntake=INTAKING, OFF+RequestedOuttake=OUTTAKING, INTAKING+RequestedOff=OFF, INTAKING+RequestedOuttake=OUTTAKING, OUTTAKING+RequestedOff=OFF, OUTTAKING+RequestedIntake=INTAKING, ANY_STATE+FaultDetected=OFF


Properties: topTunnelSpeed, bottomTunnelSpeed, topTunnelCurrentRPS, bottomTunnelCurrentRPS, hasFault, faultReason

Derived Conditions: isRunning, isIntaking, isOuttaking


Faults: TOP_MOTOR_DISCONNECTED, BOTTOM_MOTOR_DISCONNECTED, OVERCURRENT, UNKNOWN


Requirements: The tunnel shall command positive topTunnelSpeed and bottomTunnelSpeed when INTAKING.

The tunnel shall command negative topTunnelSpeed and bottomTunnelSpeed when OUTTAKING.

The tunnel shall stop both motor outputs when OFF.

The tunnel shall enter OFF if FaultDetected is received.

The tunnel shall log every state transition with timestamp, previous state, new state, and triggering event.


Acceptance Tests:
Requirement 1:
Set topTunnelSpeed to 0.35.
Set bottomTunnelSpeed to 0.35.
Trigger RequestedIntake from OFF.
Verify state becomes INTAKING.
Verify top motor output is positive topTunnelSpeed.
Verify bottom motor output is positive bottomTunnelSpeed.

Requirement 2:
Set topTunnelSpeed to 0.35.
Set bottomTunnelSpeed to 0.35.
Trigger RequestedOuttake from OFF.
Verify state becomes OUTTAKING.
Verify top motor output is negative topTunnelSpeed.
Verify bottom motor output is negative bottomTunnelSpeed.

Requirement 3:
Trigger RequestedOff from INTAKING.
Verify state becomes OFF.
Verify both motor outputs are stopped.

Requirement 4:
Trigger OFF+RequestedIntake=INTAKING.
Inspect log output.
Verify log contains:
- timestamp
- previousState = OFF
- newState = INTAKING
- event = RequestedIntake
