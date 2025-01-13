package frc.robot.subsystems;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    enum ElevatorState { // TODO: ? add algae L2 and L3 Intake States
        HOME(0, 0),
        CHUTE_INTAKE(0, 0),
        GROUND_INTAKE(0, 0),
        L1_SCORE(0, 0),
        L2_SCORE(0, 0),
        L3_SCORE(0, 0),
        L4_SCORE(0, 0),
        BARGE_SCORE(0, 0);

        /** The height of the elevator in inches. */
        private final int height;
        /** The angle of the wrist in degrees. */
        private final int angle;

        ElevatorState(int height, int angle) {
            this.height = height;
            this.angle = angle;
        }
    }

    private static ElevatorSubsystem instance;

    public static ElevatorSubsystem getInstance() {
        if (instance == null) instance = new ElevatorSubsystem();
        return instance;
    }

    /* Motors and Request */
    private ElevatorState state = ElevatorState.HOME;
    private final TalonFX leader = new TalonFX(Constants.Elevator.rightElevatorMotor, Constants.canbus);
    private final PositionTorqueCurrentFOC leaderControl;

    private final TalonFX follower = new TalonFX(Constants.Elevator.leftElevatorMotor, Constants.canbus);
    private final Follower followerControl = new Follower(leader.getDeviceID(), true);

    private final TalonFX wrist = new TalonFX(Constants.Elevator.wristMotor, Constants.canbus);
    private final PositionTorqueCurrentFOC wristControl = new PositionTorqueCurrentFOC(0);

    /* State Request Flags */
    private boolean requestHome = false;
    private boolean requestChuteIntake = false;
    private boolean requestGroundIntake = false;
    private boolean requestL1Score = false;
    private boolean requestL2Score = false;
    private boolean requestL3Score = false;
    private boolean requestL4Score = false;
    private boolean requestBargeScore = false;

    /* Other Variables */
    private boolean homedOnce = false;
    private final DigitalInput homeSwitch = new DigitalInput(Constants.Elevator.homeSwitch);

    private boolean elevatorAtPosition = false;
    private final Debouncer elevatorDebouncer = new Debouncer(0.1);
    private final StatusSignal<Angle> elevatorStatus = leader.getPosition();

    private boolean wristAtPosition = false;
    private final Debouncer wristDebouncer = new Debouncer(0.1);
    private final StatusSignal<Angle> wristStatus = wrist.getPosition();

    /**
     * Creates a new instance of this ElevatorSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ElevatorSubsystem() {
        // TODO: add configs for leader
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.Slot0.kP = 0;
        leaderConfig.Slot0.kI = 0;
        leaderConfig.Slot0.kD = 0;
        leaderConfig.Slot0.kG = 0;
        leaderConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        leaderConfig.Feedback.RotorToSensorRatio = 1; // TODO: CHANGE
        leaderConfig.Feedback.SensorToMechanismRatio = 1; // TODO: CHANGE

        leader.getConfigurator().apply(leaderConfig);
        leader.setNeutralMode(NeutralModeValue.Brake);

        leaderControl = new PositionTorqueCurrentFOC(0).withLimitForwardMotion(homeSwitch.get()); // TODO: or is it withLimitReverse motion?
        // TODO: add configs for follower
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        follower.getConfigurator().apply(followerConfig);
        follower.setNeutralMode(NeutralModeValue.Brake);

        follower.setControl(followerControl);

        // TODO: add configs for wrist
        TalonFXConfiguration wristConfig = new TalonFXConfiguration();
        wristConfig.Slot0.kP = 0;
        wristConfig.Slot0.kI = 0;
        wristConfig.Slot0.kD = 0;
        wristConfig.Slot0.kG = 0;
        wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        leaderConfig.Feedback.RotorToSensorRatio = 1;
        leaderConfig.Feedback.SensorToMechanismRatio = 1;

        wrist.getConfigurator().apply(wristConfig);
        wrist.setNeutralMode(NeutralModeValue.Brake);
    }

    private void setElevatorHeight(int height) {
        leaderControl.withPosition(height);
        leader.setControl(leaderControl);
    }

    private void setElevatorAngle(int angle) {
        wristControl.withPosition(angle);
        wrist.setControl(wristControl);
    }

    @Override
    public void periodic() {
        ElevatorState nextState = state;

        if (requestHome) nextState = ElevatorState.HOME;
        else if (requestChuteIntake) nextState = ElevatorState.CHUTE_INTAKE;
        else if (requestGroundIntake) nextState = ElevatorState.GROUND_INTAKE;
        else if (requestL1Score) nextState = ElevatorState.L1_SCORE;
        else if (requestL2Score) nextState = ElevatorState.L2_SCORE;
        else if (requestL3Score) nextState = ElevatorState.L3_SCORE;
        else if (requestL4Score) nextState = ElevatorState.L4_SCORE;

        if (nextState != state) {
            state = nextState;
            setElevatorHeight(state.height);
            setElevatorAngle(state.angle);
        }

        elevatorAtPosition = elevatorDebouncer.calculate(Math.abs(elevatorStatus.getValue().magnitude() - state.height) < 10);
        wristAtPosition = wristDebouncer.calculate(Math.abs(wristStatus.getValue().magnitude() - state.angle) < 5);

        if (homeSwitch.get()) {
            /* TODO: Force the elevator position to 0 if the home switch is pressed
              * Also STOP MOVING THE ELEVATOR
             */
            homedOnce = true;
        }

        SmartDashboard.putString("Elevator State", state.toString());
        SmartDashboard.putBoolean("Elevator At Position", elevatorAtPosition);
        SmartDashboard.putBoolean("Wrist At Position", wristAtPosition);
        SmartDashboard.putBoolean("Both At Position", elevatorAtPosition && wristAtPosition);
        SmartDashboard.putBoolean("Home Switch", homeSwitch.get());
    }

    public boolean isAtPosition() {
        return elevatorAtPosition && wristAtPosition;
    }

    private void unsetAllRequests() {
        requestHome = false;
        requestChuteIntake = false;
        requestGroundIntake = false;
        requestL1Score = false;
        requestL2Score = false;
        requestL3Score = false;
        requestL4Score = false;
        requestBargeScore = false;
    }

    public void requestHome() {
        unsetAllRequests();
        requestHome = true;
    }

    public void requestChuteIntake() {
        unsetAllRequests();
        requestChuteIntake = true;
    }

    public void requestGroundIntake() {
        unsetAllRequests();
        requestGroundIntake = true;
    }

    public void requestL1Score() {
        unsetAllRequests();
        requestL1Score = true;
    }

    public void requestL2Score() {
        unsetAllRequests();
        requestL2Score = true;
    }

    public void requestL3Score() {
        unsetAllRequests();
        requestL3Score = true;
    }

    public void requestL4Score() {
        unsetAllRequests();
        requestL4Score = true;
    }

    public void requestBargeScore() {
        unsetAllRequests();
        requestBargeScore = true;
    }
}
