package frc.robot.subsystems.elevatorwrist;


import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;

import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lights.LightsSubsystem;

public class ElevatorWristSubsystem extends SubsystemBase {
    public enum ElevatorState {
        // height is zero at the bottom of the elevator
        // angle is zero when the wrist is plumb to the ground
        HOME(0, 0, LightsSubsystem.Colors.YELLOW), // homing state, not really a position
        // idle position 90 deg
        IDLE(0, 90, LightsSubsystem.Colors.WHITE),
        CHUTE_INTAKE(0, 0, LightsSubsystem.Colors.GREEN),
        //        L1_SCORE(0, 0, LightsSubsystem.Colors.BLUE),
        L2_SCORE(30, 180, LightsSubsystem.Colors.CYAN),
        L3_SCORE(20, 0, LightsSubsystem.Colors.AQUAMARINE),
        L4_SCORE(0, 0, LightsSubsystem.Colors.PURPLE),
        CLIMB(0, 0, LightsSubsystem.Colors.PINK); // just get intake out of the way

        /**
         * The height of the elevator in inches.
         */
        private final Distance height;
        /**
         * The angle of the wrist in degrees.
         */
        private final Angle angle;

        private final LightsSubsystem.Color color;

        ElevatorState(double height, double angle, LightsSubsystem.Color color) {
            this.height = Units.Inches.of(height);
            this.angle = Units.Degrees.of(angle);
            this.color = color;
        }
    }

    public enum WristOrder {
        MOVE_FIRST,
        MOVE_LAST,
        MOVE_BOTH
    }

    /* Motors and Controls */
    private final TalonFX leaderMotor = ElevatorWristConstants.rightElevatorMotorConfig.createDevice(TalonFX::new);
    private final MotionMagicVoltage leaderControl = new MotionMagicVoltage(0);
    private final VoltageOut homeControl = new VoltageOut(0).withEnableFOC(true);
    private final TalonFX followerMotor = ElevatorWristConstants.leftElevatorMotorConfig.createDevice(TalonFX::new);
    private final Follower followerControl = new Follower(leaderMotor.getDeviceID(), true);
    private final TalonFX wristMotor = ElevatorWristConstants.wristMotorConfig.createDevice(TalonFX::new);
    private final PositionTorqueCurrentFOC wristControl = new PositionTorqueCurrentFOC(ElevatorState.HOME.angle);

    /* Sensors and Signals */
    private final Debouncer elevatorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
    private final StatusSignal<Angle> elevatorPositionStatus = leaderMotor.getPosition();
    private final StatusSignal<Current> elevatorCurrentStatus = leaderMotor.getStatorCurrent();
    private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
    private final Debouncer wristDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
    private final CANcoder wristEncoder = ElevatorWristConstants.wristEncoderConfig.createDevice(CANcoder::new);
    private final StatusSignal<Angle> wristAngleStatus = wristMotor.getPosition();
//    private final CANcoder homeCANcoder = ElevatorWristConstants.homeHallEffect.createDevice(CANcoder::new);
    private boolean elevatorStalled = false;

    /* State Machine */
    private ElevatorState state = ElevatorState.IDLE;
    private ElevatorState prevState = null;
    private WristOrder wristOrder = WristOrder.MOVE_BOTH;

    private boolean requestHome = false;
    private boolean requestIdle = false;
    private boolean requestChuteIntake = false;
    //    private boolean requestL1Score = false;
    private boolean requestL2Score = false;
    private boolean requestL3Score = false;
    private boolean requestL4Score = false;
    private boolean requestClimb = false;

    /* Other Variables */
    private boolean homedOnce = false;
    private boolean elevatorAtPosition = false;
    private boolean wristAtPosition = false;
    private final LightsSubsystem lightSubsystem = LightsSubsystem.getInstance();

//    private ElevatorWristSim sim = null;
//    private final SysIdRoutine elevatorIdRoutine = new SysIdRoutine(
//            new SysIdRoutine.Config(
//                    Units.Volts.of(0.25).per(Units.Seconds),
//                    Units.Volts.of(2),
//                    null,
//                    state -> SignalLogger.writeString("SysIdElevatorState", state.toString())
//            ),
//            new SysIdRoutine.Mechanism(
//                    (volts) -> leader.setControl(new VoltageOut(volts)),
//                    null,
//                    this
//            )
//    );

//    public Command elevatorQuasistaticId(boolean forward) {
//        return elevatorIdRoutine.quasistatic(forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
//    }
//
//    public Command elevatorDynamicId(boolean forward) {
//        return elevatorIdRoutine.dynamic(forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
//    }

    private static ElevatorWristSubsystem instance;
    public static ElevatorWristSubsystem getInstance() {
        if (instance == null) instance = new ElevatorWristSubsystem();
        return instance;
    }

    private ElevatorWristSubsystem() {
//        if (Utils.isSimulation()) sim = ElevatorWristSim.getInstance();

        leaderMotor.setControl(leaderControl);

        followerMotor.setControl(followerControl);

        wristMotor.setControl(wristControl);

        leaderMotor.setPosition(0);
    }

    private void setElevatorHeight(Distance height) {
        leaderControl.withPosition(height.timesConversionFactor(ElevatorWristConstants.revolutionsPerInch));
        if (state != ElevatorState.HOME) leaderMotor.setControl(leaderControl);
    }

    private void setWristAngle(Angle angle) {
        wristControl.withPosition(angle).withSlot(IntakeSubsystem.getInstance().coralDetected() ? 1 : 0);
        wristMotor.setControl(wristControl);
    }
    
    public void setBrakeMode() {
        leaderMotor.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);

        wristMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setCoastMode() {
        leaderMotor.setNeutralMode(NeutralModeValue.Coast);
        followerMotor.setNeutralMode(NeutralModeValue.Coast);

        wristMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void periodic() {
        ElevatorState nextState = getNextState();

        if (state != ElevatorState.HOME) {
            if (state != nextState) {
                lightSubsystem.requestColor(nextState.color);
                elevatorAtPosition = elevatorDebouncer.calculate(false);
                wristAtPosition = wristDebouncer.calculate(false);
                state = nextState;
                unsetAllRequests();
            }

            if (state == ElevatorState.HOME) homeElevator(); // special case for homing
            else {
                if ((wristOrder != WristOrder.MOVE_FIRST || wristAngleStatus.getValue().gt(Units.Revolutions.of(0.35)))) { // scuffed logic to make sure the elevator doesn't move before the wrist
                    setElevatorHeight(state.height);
                }
            }
            if ((wristOrder != WristOrder.MOVE_LAST || elevatorAtPosition)) { // scuffed logic to make sure the wrist doesn't move before the elevator
                setWristAngle(state.angle);
            }
        }
        homingPeriodic(); // TODO: test homing

        elevatorPositionStatus.refresh(false);
        elevatorCurrentStatus.refresh(false);
        wristAngleStatus.refresh(false);

        elevatorAtPosition = elevatorDebouncer.calculate(
             elevatorPositionStatus.getValue().isNear(
             state.height.timesConversionFactor(ElevatorWristConstants.revolutionsPerInch)
             , Units.Revolutions.of(0.2))
        ); // 0.2 revolutions tolerance
        wristAtPosition = wristDebouncer.calculate(wristAngleStatus.getValue().isNear(state.angle, 0.02)); // 0.02 revolutions tolerance
        elevatorStalled = Math.abs(currentFilter.calculate(elevatorCurrentStatus.getValueAsDouble())) > 20;

        telemetry();

        if (elevatorAtPosition && wristAtPosition) prevState = state;
    }

    private void homeElevator() {
        // Force the elevator to move down until the home switch or current limit is reached is triggered
        homeControl.withOutput(-1.0); // kG ~ 0.35
        leaderMotor.setControl(homeControl);
    }

    /**
     * Sets the elevator to the home position.
     * This is used for homing the elevator. <b>(ONLY USE IN CASE OF EMERGENCY)</b>
     */
    public void setElevatorZero() {
        leaderMotor.setPosition(0);
    }

    private void homingPeriodic() {
        setWristAngle(ElevatorState.CLIMB.angle);
        if (getHomeCANcoder() || state == ElevatorState.HOME && elevatorStalled) {
            homedOnce = true;
            leaderMotor.setPosition(0);
            if (state == ElevatorState.HOME) {
                requestIdle(WristOrder.MOVE_BOTH);
                System.out.println("At Home Position: " + (getHomeCANcoder() ? "Home Switch" : "Current"));
            }
        }
    }

    private ElevatorState getNextState() {
        ElevatorState nextState = state;

        if (requestHome) nextState = ElevatorState.HOME;
        else if (requestIdle) nextState = ElevatorState.IDLE;
        else if (requestChuteIntake) nextState = ElevatorState.CHUTE_INTAKE;
//        else if (requestL1Score) nextState = ElevatorState.L1_SCORE;
        else if (requestL2Score) nextState = ElevatorState.L2_SCORE;
        else if (requestL3Score) nextState = ElevatorState.L3_SCORE;
        else if (requestL4Score) nextState = ElevatorState.L4_SCORE;
        else if (requestClimb) nextState = ElevatorState.CLIMB;

        if (nextState != state) prevState = state;
        return nextState;
    }

    private boolean getHomeCANcoder() {
        return false;
//        return homeCANcoder.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Red;
    }

    public boolean isAtPosition() {
        return elevatorAtPosition && wristAtPosition;
    }

    public boolean isTall() {
        return elevatorPositionStatus.getValue().gt(Units.Inches.of(6).timesConversionFactor(ElevatorWristConstants.revolutionsPerInch)); // if the elevator is taller than 6 inches
    }

    public boolean homedOnce() {
        return homedOnce;
    }

    public double movePercent() {
        if (prevState == null) return 0;
        return Math.hypot(state.height.minus(prevState.height).magnitude(), state.angle.minus(prevState.angle).magnitude());
    }

//    @Override
//    public void simulationPeriodic() {
//        sim.update(state.name(), state.height, state.angle);
//    }

    private void unsetAllRequests() {
        requestHome = false;
        requestIdle = false;
        requestChuteIntake = false;
//        requestL1Score = false;
        requestL2Score = false;
        requestL3Score = false;
        requestL4Score = false;
        requestClimb = false;
    }

    public void requestHome(WristOrder order) {
        unsetAllRequests();
        requestHome = true;
        wristOrder = order;
    }

    public void requestIdle(WristOrder order) {
        unsetAllRequests();
        requestIdle = true;
        wristOrder = order;
    }

    public void requestChuteIntake(WristOrder order) {
        unsetAllRequests();
        requestChuteIntake = true;
        wristOrder = order;
    }

//    public void requestL1Score(WristOrder order) {
//        unsetAllRequests();
//        requestL1Score = true;
//        wristOrder = order;
//    }

    public void requestL2Score(WristOrder order) {
        unsetAllRequests();
        requestL2Score = true;
        wristOrder = order;
    }

    public void requestL3Score(WristOrder order) {
        unsetAllRequests();
        requestL3Score = true;
        wristOrder = order;
    }

    public void requestL4Score(WristOrder order) {
        unsetAllRequests();
        requestL4Score = true;
        wristOrder = order;
    }

    public void requestClimb(WristOrder order) {
        unsetAllRequests();
        requestClimb = true;
        wristOrder = order;
    }

    private void telemetry() {
        SmartDashboard.putString("ElevatorWrist/Elevator State", state.toString());
        SmartDashboard.putString("ElevatorWrist/Prev Elevator State", prevState != null ? prevState.toString() : "null");
        SmartDashboard.putNumber("ElevatorWrist/MovePercent", movePercent());

        SmartDashboard.putNumber("ElevatorWrist/Elevator Height", elevatorPositionStatus.getValueAsDouble());
        SmartDashboard.putNumber("ElevatorWrist/Elevator Current", elevatorCurrentStatus.getValueAsDouble());
        SmartDashboard.putNumber("ElevatorWrist/Elevator Setpoint", state.height.in(Units.Inches) *
                ElevatorWristConstants.revolutionsPerInch.in(PerUnit.combine(Units.Revolutions, Units.Inches)));
        SmartDashboard.putNumber("ElevatorWrist/Wrist Angle", wristAngleStatus.getValue().in(Units.Revolutions));
        SmartDashboard.putNumber("ElevatorWrist/Wrist Setpoint", state.angle.in(Units.Revolutions));

        SmartDashboard.putBoolean("ElevatorWrist/Homed Once", homedOnce);
        SmartDashboard.putBoolean("ElevatorWrist/Homing", state == ElevatorState.HOME);
        SmartDashboard.putBoolean("ElevatorWrist/Home Switch", getHomeCANcoder());
        SmartDashboard.putBoolean("ElevatorWrist/Elevator At Position", elevatorAtPosition);
        SmartDashboard.putBoolean("ElevatorWrist/Wrist At Position", wristAtPosition);
        SmartDashboard.putBoolean("ElevatorWrist/Both At Position", elevatorAtPosition && wristAtPosition);
    }
}
