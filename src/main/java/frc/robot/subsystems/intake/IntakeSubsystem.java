package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lights.LightsSubsystem;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;

    /* Motors */
    private final TalonFX intakeMotor = IntakeConstants.intakeMotorConfig.createDevice(TalonFX::new);
    private final VoltageOut intakeControl = new VoltageOut(0);

    /* Sensors */
    private final CANrange coralDistanceSensor = IntakeConstants.coralSensorConfig.createDevice(CANrange::new);

    /* Statuses */
    private boolean coralDetected = false;
    private final StatusSignal<Boolean> coralDetectedSignal = coralDistanceSensor.getIsDetected();

    private boolean stalled = false;
//    private final StatusSignal<Current> currentSignal = intakeMotor.getStatorCurrent();
    private final LinearFilter currentFilter = LinearFilter.movingAverage(5);

    /* State Machine Logic */
    private enum IntakeState {
        IDLE,
        INTAKING,
        SPITTING
    }
    private IntakeState state = IntakeState.IDLE;

    private boolean requestedIdle = false;
    private boolean requestedIntake = false;
    private boolean requestedSpit = false;

    public static IntakeSubsystem getInstance() {
        if (instance == null) instance = new IntakeSubsystem();
        return instance;
    }

    private IntakeSubsystem() {
        setIntakeMotor(0);
    }

    /**
     * Set the intake motor to a given speed
     * @param voltage in volts
     */
    private void setIntakeMotor(double voltage) {
       intakeMotor.setControl(intakeControl.withOutput(voltage));
    }

    @Override
    public void periodic() {
        IntakeState nextState = state;
        if (requestedIdle) nextState = IntakeState.IDLE;
        else if (requestedIntake) nextState = IntakeState.INTAKING;
        else if (requestedSpit) nextState = IntakeState.SPITTING;

        if (nextState != state) {
            state = nextState;
            unsetAllRequests();

            switch (state) {
                case IDLE -> {
                    if(coralDetected) setIntakeMotor(2);
                    else setIntakeMotor(0);
                }
                case INTAKING -> setIntakeMotor(IntakeConstants.intakeSpeed);
                case SPITTING -> setIntakeMotor(-IntakeConstants.spitSpeed);
            }
            LightsSubsystem.getInstance().requestBlinking(state != IntakeState.IDLE);
        }

        if (state == IntakeState.INTAKING && coralDetected) {
            requestIdle();
            System.out.println("IntakeSubsystem: Stopping intake due to: coral beam broken");
        }

        coralDetectedSignal.refresh(false);
        coralDetected = coralDetectedSignal.getValue();
//        currentSignal.refresh(false);
//        stalled = currentFilter.calculate(currentSignal.getValue().in(Units.Amps)) > IntakeConstants.stalledCurrentThreshold; // might be another way to detect game pieces

        SmartDashboard.putBoolean("Intake/Coral Detected", coralDetected);
        // SmartDashboard.putBoolean("Intake/Stalled", stalled);
        SmartDashboard.putNumber("Intake/Intake Speed", intakeMotor.getVelocity().getValueAsDouble());
    }

    public boolean coralDetected() {
        return coralDetected;
    }

    private void unsetAllRequests() {
        requestedIdle = false;
        requestedIntake = false;
        requestedSpit = false;
    }

    public void requestIdle() {
        unsetAllRequests();
        requestedIdle = true;
    }

    public void requestIntake() {
        unsetAllRequests();
        requestedIntake = true;
    }

    public void requestSpit() {
        unsetAllRequests();
        requestedSpit = true;
    }
}

