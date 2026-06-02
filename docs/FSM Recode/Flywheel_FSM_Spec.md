Mission: The flywheel is responsible for launching fuel to the commanded and desired launch velocity required for accurate and consistent scoring.


States: IDLE, READY, FAULT, SPINNING_UP, SHOOTING


Events: RequestedShoot , SpinupTimeout , FaultDetected , TargetRPMReached, ShotStarted, RequestedReset, ShotCompleted


Transitions: IDLE+RequestedShoot=SPINNING_UP, SPINNING_UP+TargetRPMReached=READY, SPINNING_UP +SpinupTimeout=FAULT, ANY_STATE+FaultDetected= FAULT, READY+ShotStarted=SHOOTING, SHOOTING+ShotCompleted=IDLE, FAULT+RequestedReset=IDLE


Properties: targetRPM, currentRPM, rpmError, hasFault, faultReason

Derived Conditions: isAtTargetRPM, canShoot


Faults: MOTOR_DISCONNECTED, ENCODER_DISCONNECTED, SPINUP_TIMEOUT, RPM_UNSTABLE, UNKNOWN


Requirements: The flywheel shall reach target RPM within 1.0 second of RequestedShoot.

The flywheel shall enter FAULT if target RPM is not reached within 2.0 seconds.

The flywheel shall log every state transition with timestamp, previous state, new state, and triggering event.


Acceptance Tests:
Requirement 1:
Set targetRPS to 83.
Trigger RequestedShoot from IDLE.
Verify state becomes SPINNING_UP.
Verify currentRPM reaches targetRPM tolerance within 1.0 second.
Verify state becomes READY.

Requirement 2:
Set targetRPS to 83.
Trigger RequestedShoot from IDLE.
Simulate currentRPM staying below tolerance for 2.0 seconds.
Verify state becomes FAULT.
Verify faultReason = SPINUP_TIMEOUT.

Requirement 3:
Trigger IDLE → SPINNING_UP.
Inspect log output.
Verify log contains:
- timestamp
- previousState = IDLE
- newState = SPINNING_UP
- event = ShootRequested