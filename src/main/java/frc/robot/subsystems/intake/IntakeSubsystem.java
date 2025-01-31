package frc.robot.subsystems.intake;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;

    /* Motors */
    private final TalonFX intakeMotor = IntakeConstants.intakeMotorConfig.createMotor();
    private final VelocityTorqueCurrentFOC intakeControl = new VelocityTorqueCurrentFOC(0);

    /* Sensors */
    private final DigitalInput coralBeamBreak = new DigitalInput(IntakeConstants.beamBreakPort);
    private final CANrange algaeDistanceSensor = new CANrange(IntakeConstants.distanceSensorID);

    /* Statuses */
    private boolean coralBeamBroken = false;
    private final Debouncer coralBeamBreakDebouncer = new Debouncer(0.1);

    private final StatusSignal<Distance> algaeDistance = algaeDistanceSensor.getDistance();
    private boolean algaeDetected = false;

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
     * @param speed in rev/s
     */
    private void setIntakeMotor(double speed) {
        intakeMotor.setControl(intakeControl.withVelocity(speed));
    }

    @Override
    public void periodic() {
        IntakeState nextState = state;
        if (requestedIdle) nextState = IntakeState.IDLE;
        else if (requestedIntake) nextState = IntakeState.INTAKING;
        else if (requestedSpit) nextState = IntakeState.SPITTING;

        if (nextState != state) {
            switch (nextState) {
                case IDLE -> setIntakeMotor(0);
                case INTAKING -> setIntakeMotor(IntakeConstants.intakeSpeed);
                case SPITTING -> setIntakeMotor(-IntakeConstants.spitSpeed);
            }
            state = nextState;
        }

        if (state == IntakeState.INTAKING && (coralBeamBroken || algaeDetected)) {
            setIntakeMotor(0);
            state = IntakeState.IDLE;
        }

        // This method will be called once per scheduler run
        coralBeamBroken = coralBeamBreakDebouncer.calculate(coralBeamBreak.get());
        algaeDistance.refresh(); // TODO: Run all signals in signal thread?
        algaeDetected = algaeDistance.getValueAsDouble() < IntakeConstants.algaeDistanceThreshold;

        SmartDashboard.putBoolean("Intake/Coral Beam Broken", coralBeamBroken);
        SmartDashboard.putBoolean("Intake/Algae Detected", algaeDetected);
        SmartDashboard.putNumber("Intake/Algae Distance", algaeDistance.getValueAsDouble());
        SmartDashboard.putNumber("Intake/Intake Speed", intakeMotor.getVelocity().getValueAsDouble());
    }

    public boolean coralBeamBroken() {
        return coralBeamBroken;
    }

    public boolean algaeDetected() {
        return algaeDetected;
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

