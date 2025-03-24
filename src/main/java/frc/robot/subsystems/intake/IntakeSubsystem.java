package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lights.LightsSubsystem;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;

    /* Motors */
//    private final TalonFX intakeMotor = IntakeConstants.intakeMotorConfig.createDevice(TalonFX::new);
    private final VoltageOut intakeControl = new VoltageOut(0); // TODO: Tempted to use dutyCycle, but that will not adapt to forces as much

    /* Sensors */
    private final DigitalInput coralBeamBreak = new DigitalInput(IntakeConstants.beamBreakPort);
    // private final CANrange algaeDistanceSensor = IntakeConstants.distanceSensorConfig.createDevice(CANrange::new);

    /* Statuses */
    private boolean coralBeamBroken = false;
    private final Debouncer coralBeamBreakDebouncer = new Debouncer(0.1);

    private boolean algaeDetected = false;
    // private final StatusSignal<Distance> algaeDistanceSignal = algaeDistanceSensor.getDistance();

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
//        intakeMotor.setControl(intakeControl.withOutput(voltage));
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
                case IDLE -> setIntakeMotor(2);
                case INTAKING -> setIntakeMotor(IntakeConstants.intakeSpeed);
                case SPITTING -> setIntakeMotor(-IntakeConstants.spitSpeed);
            }
            LightsSubsystem.getInstance().requestBlinking(state != IntakeState.IDLE);
        }

        // if (state == IntakeState.INTAKING && (coralBeamBroken || algaeDetected)) {
        //     setIntakeMotor(0);
        //     state = IntakeState.IDLE;

        //     System.out.println("IntakeSubsystem: Stopping intake due to: " + (coralBeamBroken ? "coral beam broken" : "algae detected"));
        // }

        coralBeamBroken = coralBeamBreakDebouncer.calculate(!coralBeamBreak.get());
        // algaeDistanceSignal.refresh(false);
        // algaeDetected = algaeDistanceSignal.getValueAsDouble() < IntakeConstants.algaeDistanceThreshold;
//        currentSignal.refresh(false);
//        stalled = currentFilter.calculate(currentSignal.getValue().in(Units.Amps)) > IntakeConstants.stalledCurrentThreshold; // might be another way to detect game pieces

        SmartDashboard.putBoolean("Intake/Coral Beam Broken", coralBeamBroken);
        SmartDashboard.putBoolean("Intake/Algae Detected", algaeDetected);
        SmartDashboard.putBoolean("Intake/Stalled", stalled);
        // SmartDashboard.putNumber("Intake/Algae Distance", algaeDistanceSignal.getValueAsDouble());
//        SmartDashboard.putNumber("Intake/Intake Speed", intakeMotor.getVelocity().getValueAsDouble());
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

    public boolean isDone(){
        return state == IntakeState.IDLE || state == IntakeState.SPITTING;
    }
}

