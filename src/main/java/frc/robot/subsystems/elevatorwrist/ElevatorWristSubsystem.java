package frc.robot.subsystems.elevatorwrist;


import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.simulation.ElevatorWristSim;

public class ElevatorWristSubsystem extends SubsystemBase {
    private static ElevatorWristSubsystem instance;

    enum ElevatorState { // TODO: ? add algae L2 and L3 Intake States
        // height is zero at the bottom of the elevator
        // angle is zero when the wrist is plumb to the ground
        HOME(0, 0, LEDSubsystem.Colors.WHITE),
        CHUTE_INTAKE(0, 0, LEDSubsystem.Colors.GREEN),
        GROUND_INTAKE(0, 0, LEDSubsystem.Colors.YELLOW),
        L1_SCORE(0, 0, LEDSubsystem.Colors.BLUE),
        L2_SCORE(0, 0, LEDSubsystem.Colors.CYAN),
        L3_SCORE(0, 0, LEDSubsystem.Colors.AQUAMARINE),
        L4_SCORE(0, 0, LEDSubsystem.Colors.PERSIAN_BLUE),
        BARGE_SCORE(0, 0, LEDSubsystem.Colors.PURPLE);

        /**
         * The height of the elevator in inches.
         */
        private final int height;
        /**
         * The angle of the wrist in degrees.
         */
        private final int angle;

        private final LEDSubsystem.Color color;

        ElevatorState(int height, int angle, LEDSubsystem.Color color) {
            this.height = height;
            this.angle = angle;
            this.color = color;
        }
    }

    /* Motors and Controls */
//    private final TalonFX leader = ElevatorWristConstants.rightElevatorMotorConfig.createMotor();
//    private final PositionTorqueCurrentFOC leaderControl = new PositionTorqueCurrentFOC(0);
//
//    private final TalonFX follower = ElevatorWristConstants.leftElevatorMotorConfig.createMotor();
//    private final Follower followerControl = new Follower(leader.getDeviceID(), true);
//
//    private final TalonFX wrist = ElevatorWristConstants.wristMotorConfig.createMotor();
//    private final PositionTorqueCurrentFOC wristControl = new PositionTorqueCurrentFOC(0);

    /* Sensors and Triggers */
//    private final Trigger homeTrigger = new Trigger(this::getHomeCANcoder);

//    private final Debouncer elevatorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
//    private final StatusSignal<Angle> elevatorStatus = leader.getPosition();
//
//    private final CANcoder homeCANcoder = new CANcoder(ElevatorWristConstants.homeCANcoderID);
//    // private final DigitalInput homeSwitch = new DigitalInput(ElevatorWristConstants.homeCANcoderID);
//
//    private final Debouncer wristDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
//    private final CANcoder wristEncoder = new CANcoder(ElevatorWristConstants.wristEncoderID);
//    private final StatusSignal<Angle> wristStatus = wristEncoder.getPosition();

    /* State Machine Logic */
    private ElevatorState state = ElevatorState.HOME;

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
    private boolean elevatorAtPosition = false;
    private boolean wristAtPosition = false;
    private final LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();

    private ElevatorWristSim sim = null;

    private ElevatorWristSubsystem() {
        if (Utils.isSimulation()) sim = ElevatorWristSim.getInstance();

        // TODO: add configs for leader in ElevatorWristConstants
//        leader.setNeutralMode(NeutralModeValue.Brake);
//        leader.setControl(leaderControl);
//
//        homeTrigger.onTrue(new InstantCommand(() -> {
//         leader.setPosition(0);
//         //leader.setControl(leaderControl.withPosition(0)); // This will definitely cause issues, instead try to prevent forward motion
//         homedOnce = true;
//        }));

        // TODO: add configs for follower in ElevatorWristConstants
//        follower.setNeutralMode(NeutralModeValue.Brake);
//        follower.setControl(followerControl);

        // TODO: add configs for wrist in ElevatorWristConstants
//        wrist.setNeutralMode(NeutralModeValue.Brake);
//        wrist.setControl(wristControl);

//        wristEncoder.getConfigurator().apply(ElevatorWristConstants.wristEncoderConfig);
    }

    public static ElevatorWristSubsystem getInstance() {
        if (instance == null) instance = new ElevatorWristSubsystem();
        return instance;
    }

    private void homeElevator() {
//        leaderControl.withPosition(-10).withVelocity(1).withLimitForwardMotion(true);
//        // TODO: force the elevator to move down until the home switch is pressed at a slower speed for safety
//        leader.setControl(leaderControl);
    }

    private void setElevatorHeight(int height) {
//        leaderControl.withPosition(height).withVelocity(0);
//        leader.setControl(leaderControl);
    }

    private void setElevatorAngle(int angle) {
//        wristControl.withPosition(angle);
//        wrist.setControl(wristControl);
    }

    @Override
    public void periodic() {
        ElevatorState nextState = state;

        if (requestHome || !homedOnce) nextState = ElevatorState.HOME;
        else if (requestChuteIntake) nextState = ElevatorState.CHUTE_INTAKE;
        else if (requestGroundIntake) nextState = ElevatorState.GROUND_INTAKE;
        else if (requestL1Score) nextState = ElevatorState.L1_SCORE;
        else if (requestL2Score) nextState = ElevatorState.L2_SCORE;
        else if (requestL3Score) nextState = ElevatorState.L3_SCORE;
        else if (requestL4Score) nextState = ElevatorState.L4_SCORE;

        if (nextState != state) {
            state = nextState;
            if (state == ElevatorState.HOME) homeElevator();
            else setElevatorHeight(state.height);
            setElevatorAngle(state.angle);
            ledSubsystem.requestColor(state.color);
        }

//        elevatorStatus.refresh(); // TODO: Run all signals in signal thread?
//        wristStatus.refresh(); // TODO: Run all signals in signal thread?
//
//        elevatorAtPosition = elevatorDebouncer.calculate(Math.abs(elevatorStatus.getValueAsDouble() - state.height) < 10);
//        wristAtPosition = wristDebouncer.calculate(Math.abs(wristStatus.getValueAsDouble() - state.angle) < 5);
//
//        SmartDashboard.putString("Elevator State", state.toString());
//        SmartDashboard.putNumber("Elevator Setpoint", state.height);
//        SmartDashboard.putNumber("Wrist Setpoint", state.angle);
//
//        SmartDashboard.putNumber("Elevator Height", elevatorStatus.getValueAsDouble());
//        SmartDashboard.putNumber("Wrist Angle", wristStatus.getValueAsDouble());

        SmartDashboard.putBoolean("Homed Once", homedOnce);
        SmartDashboard.putBoolean("Elevator At Position", elevatorAtPosition);
        SmartDashboard.putBoolean("Wrist At Position", wristAtPosition);
        SmartDashboard.putBoolean("Home Switch", getHomeCANcoder());
        SmartDashboard.putBoolean("Both At Position", elevatorAtPosition && wristAtPosition);
    }

    @Override
    public void simulationPeriodic() {
        sim.update(state.name(), state.height, state.angle);
    }

    private boolean getHomeCANcoder() {
//        return homeCANcoder.getMagnetHealth().getValue() == MagnetHealthValue.Magnet_Red;
        return false;
    }

    public boolean isAtPosition() {
        return elevatorAtPosition && wristAtPosition;
    }

    public void setBrakeMode() {
//        leader.setNeutralMode(NeutralModeValue.Brake);
//        follower.setNeutralMode(NeutralModeValue.Brake);
//
//        wrist.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setCoastMode() {
//        leader.setNeutralMode(NeutralModeValue.Coast);
//        follower.setNeutralMode(NeutralModeValue.Coast);
//
//        wrist.setNeutralMode(NeutralModeValue.Coast);
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
