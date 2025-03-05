package frc.robot.subsystems.elevatorwrist;


import com.ctre.phoenix6.Utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.lights.LightsSubsystem;
import frc.robot.subsystems.simulation.ElevatorWristSim;

public class ElevatorWristSubsystem extends SubsystemBase {
    private static ElevatorWristSubsystem instance;

    enum ElevatorState { // TODO: ? add algae L2 and L3 Intake States
        // height is zero at the bottom of the elevator
        // angle is zero when the wrist is plumb to the ground
        HOME(0, 0, LightsSubsystem.Colors.WHITE),
        CHUTE_INTAKE(0, 0, LightsSubsystem.Colors.GREEN),
        GROUND_INTAKE(0, 0, LightsSubsystem.Colors.YELLOW),
        L1_SCORE(0, 0, LightsSubsystem.Colors.BLUE),
        L2_SCORE(0, 0, LightsSubsystem.Colors.CYAN),
        L3_SCORE(0, 0, LightsSubsystem.Colors.AQUAMARINE),
        L4_SCORE(0, 0, LightsSubsystem.Colors.PERSIAN_BLUE),
        L2_INTAKE(0, 0, LightsSubsystem.Colors.ORANGE),
        L3_INTAKE(0, 0, LightsSubsystem.Colors.PINK),
        BARGE_SCORE(0, 0, LightsSubsystem.Colors.PURPLE);

        /**
         * The height of the elevator in inches.
         */
        private final Distance height;
        /**
         * The angle of the wrist in degrees.
         */
        private final Angle angle;

        private final LightsSubsystem.Color color;

        ElevatorState(int height, int angle, LightsSubsystem.Color color) {
            this.height = Units.Inches.of(height);
            this.angle = Units.Degrees.of(angle);
            this.color = color;
        }
    }

    /* Motors and Controls */
//    private final TalonFX leader = ElevatorWristConstants.rightElevatorMotorConfig.createDevice(TalonFX::new);
//    private final PositionTorqueCurrentFOC leaderControl = new PositionTorqueCurrentFOC(0);
//    private final VoltageOut homeControl = new VoltageOut(0).withEnableFOC(true);
//
//    private final TalonFX follower = ElevatorWristConstants.leftElevatorMotorConfig.createDevice(TalonFX::new);
//    private final Follower followerControl = new Follower(leader.getDeviceID(), true);
//
//    private final TalonFX wrist = ElevatorWristConstants.wristMotorConfig.createDevice(TalonFX::new);
//    private final PositionTorqueCurrentFOC wristControl = new PositionTorqueCurrentFOC(0);

    /* Sensors and Signals */
//    private final Debouncer elevatorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
//    private final StatusSignal<Angle> elevatorPositionStatus = leader.getPosition();
//    private final StatusSignal<Current> elevatorCurrentStatus = leader.getStatorCurrent();
//    private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
//
//    private final CANcoder homeCANcoder = ElevatorWristConstants.homeHallEffect.createDevice(CANcoder::new);
//
//    private final Debouncer wristDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
//    private final CANcoder wristEncoder = ElevatorWristConstants.wristEncoderConfig.createDevice(CANcoder::new);
//    private final StatusSignal<Angle> wristAngleStatus = wristEncoder.getPosition();

    /* State Machine Logic */
    private ElevatorState prevState = ElevatorState.HOME;
    private ElevatorState state = ElevatorState.HOME;

    private boolean requestHome = true;
    private boolean requestChuteIntake = false;
    private boolean requestGroundIntake = false;
    private boolean requestL1Score = false;
    private boolean requestL2Score = false;
    private boolean requestL3Score = false;
    private boolean requestL4Score = false;
    private boolean requestBargeScore = false;

    /* Other Variables */
    private boolean homedOnce = false;
    private boolean homing = false;
    private boolean elevatorAtPosition = false;
    private boolean wristAtPosition = false;

//    private final LightsSubsystem lightSubsystem = LightsSubsystem.getInstance();

    private ElevatorWristSim sim = null;

//    private final SysIdRoutine elevatorIdRoutine = new SysIdRoutine(
//        new SysIdRoutine.Config(
//            null,
//            Volts.of(4),
//            null,
//            state -> SignalLogger.writeString("SysIdElevatorState", state.toString())
//        ),
//        new SysIdRoutine.Mechanism(
//            (volts) -> leader.setControl(new VoltageOut(volts.in(Volts))),
//            null,
//            this
//        )
//    );

//    private final SysIdRoutine wristIdRoutine = new SysIdRoutine(
//        new SysIdRoutine.Config(
//            null,
//            Volts.of(4),
//            null,
//            state -> SignalLogger.writeString("SysIdWristState", state.toString())
//        ),
//        new SysIdRoutine.Mechanism(
//            (volts) -> wrist.setControl(new VoltageOut(volts.in(Volts))),
//            null,
//            this
//        )
//    );

    public Command elevatorQuasistaticId(boolean forward) {
        return new InstantCommand();
//        return elevatorIdRoutine.quasistatic(forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
    }

    public Command elevatorDynamicId(boolean forward) {
        return new InstantCommand();
//        return elevatorIdRoutine.dynamic(forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
    }

    public Command wristQuasistaticId(boolean forward) {
        return new InstantCommand();
//        return wristIdRoutine.quasistatic(forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
    }

    public Command wristDynamicId(boolean forward) {
        return new InstantCommand();
//        return wristIdRoutine.dynamic(forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
    }

    public static ElevatorWristSubsystem getInstance() {
        if (instance == null) instance = new ElevatorWristSubsystem();
        return instance;
    }

    private ElevatorWristSubsystem() {
        if (Utils.isSimulation()) sim = ElevatorWristSim.getInstance();

//        // TODO: add configs for leader in ElevatorWristConstants
//        leader.setNeutralMode(NeutralModeValue.Brake);
//        leader.setControl(leaderControl);
//
//        // TODO: add configs for follower in ElevatorWristConstants
//        follower.setNeutralMode(NeutralModeValue.Brake);
//        follower.setControl(followerControl);
//
//        // TODO: add configs for wrist in ElevatorWristConstants
//        wrist.setNeutralMode(NeutralModeValue.Brake);
//        wrist.setControl(wristControl);
//
//        CTREUtil.applyConfiguration(wristEncoder, ElevatorWristConstants.wristEncoderConfig);
    }

    private void homeElevator() {
        // TODO: force the elevator to move down until the home switch is trigger at a slower speed for safety
//        homeControl.withOutput(-0.5);
//        homing = true;
//        leader.setControl(homeControl);
    }

    private void setElevatorHeight(Distance height) {
//        leaderControl.withPosition(Revolutions.of(height.in(Units.Inches) * ElevatorWristConstants.revolutionsPerInch)).withVelocity(0);
//        if (!homing) leader.setControl(leaderControl);
    }

    private void setElevatorAngle(Angle angle) {
//        wristControl.withPosition(angle);
//        wrist.setControl(wristControl);
    }

    @Override
    public void periodic() {
        ElevatorState nextState = getNextState();

        if (nextState != state && !homing) {
            state = nextState;
            unsetAllRequests();

            if (state == ElevatorState.HOME) homeElevator();
            else setElevatorHeight(state.height);
            setElevatorAngle(state.angle);

//            lightSubsystem.requestColor(state.color);
        }

//        elevatorPositionStatus.refresh(); // TODO: Run all signals in signal thread?
//        elevatorCurrentStatus.refresh(); // TODO: Run all signals in signal thread?
//        wristAngleStatus.refresh(); // TODO: Run all signals in signal thread?
//
//        elevatorAtPosition = elevatorDebouncer.calculate(Math.abs(elevatorPositionStatus.getValueAsDouble() - state.height.magnitude()) < 10);
//        wristAtPosition = wristDebouncer.calculate(Math.abs(wristAngleStatus.getValueAsDouble() - state.angle.magnitude()) < 5);

        homingExecute();

        SmartDashboard.putString("Elevator State", state.toString());
        //SmartDashboard.putString("Prev Elevator State", prevState.toString());
        SmartDashboard.putNumber("Elevator Setpoint", state.height.magnitude());
        SmartDashboard.putNumber("Wrist Setpoint", state.angle.magnitude());

//        SmartDashboard.putNumber("Elevator Height", elevatorPositionStatus.getValueAsDouble());
//        SmartDashboard.putNumber("Elevator Current", elevatorCurrentStatus.getValueAsDouble());
//        SmartDashboard.putNumber("Wrist Angle", wristAngleStatus.getValueAsDouble());

        SmartDashboard.putBoolean("Homed Once", homedOnce);
        SmartDashboard.putBoolean("Elevator At Position", elevatorAtPosition);
        SmartDashboard.putBoolean("Wrist At Position", wristAtPosition);
        SmartDashboard.putBoolean("Home Switch", getHomeCANcoder());
        SmartDashboard.putBoolean("Both At Position", elevatorAtPosition && wristAtPosition);

        if (elevatorAtPosition && wristAtPosition) prevState = state;
    }

    private void homingExecute() {
//        if (getHomeCANcoder() ||
//            currentFilter.calculate(elevatorCurrentStatus.getValueAsDouble()) < 20) {
//            homing = false;
//            homedOnce = true;
//            leader.setPosition(0);
//            System.out.println("At Home Position: " + (getHomeCANcoder() ? "Home Switch" : "Current"));
//        }
    }

    private ElevatorState getNextState() {
        ElevatorState nextState = state;

        if (requestHome) nextState = ElevatorState.HOME;
        else if (requestChuteIntake) nextState = ElevatorState.CHUTE_INTAKE;
        else if (requestGroundIntake) nextState = ElevatorState.GROUND_INTAKE;
        else if (requestL1Score) nextState = ElevatorState.L1_SCORE;
        else if (requestL2Score) nextState = ElevatorState.L2_SCORE;
        else if (requestL3Score) nextState = ElevatorState.L3_SCORE;
        else if (requestL4Score) nextState = ElevatorState.L4_SCORE;
        else if (requestBargeScore) nextState = ElevatorState.BARGE_SCORE;
        if (nextState != state) prevState = state;
        return nextState;
    }

    @Override
    public void simulationPeriodic() {
        sim.update(state.name(), state.height, state.angle);
    }

    private boolean getHomeCANcoder() {
        return false;
//        return homeCANcoder.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Red;
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

    public double movePercent() {
        return Math.hypot(state.height.minus(prevState.height).magnitude(), state.angle.minus(prevState.angle).magnitude());
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
