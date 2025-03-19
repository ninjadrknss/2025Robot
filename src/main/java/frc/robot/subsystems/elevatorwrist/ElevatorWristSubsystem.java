package frc.robot.subsystems.elevatorwrist;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.lights.LightsSubsystem;

public class ElevatorWristSubsystem extends SubsystemBase {
    public enum ElevatorState {
        // height is zero at the bottom of the elevator
        // angle is zero when the wrist is plumb to the ground
        HOME(0, 0, LightsSubsystem.Colors.YELLOW),
        // idle position -90 deg
        IDLE(0, -90, LightsSubsystem.Colors.WHITE),
        CHUTE_INTAKE(0, -20, LightsSubsystem.Colors.GREEN),
        //        L1_SCORE(0, 0, LightsSubsystem.Colors.BLUE),
        L2_SCORE(4, -235, LightsSubsystem.Colors.CYAN),
        L3_SCORE(20, 0, LightsSubsystem.Colors.AQUAMARINE),
        //        L4_SCORE(0, 0, LightsSubsystem.Colors.PERSIAN_BLUE),
        CLIMB(0, 0, LightsSubsystem.Colors.PURPLE);

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
    private final TalonFX leader = ElevatorWristConstants.rightElevatorMotorConfig.createDevice(TalonFX::new);
    // private final PositionTorqueCurrentFOC leaderControl = new PositionTorqueCurrentFOC(0);
    private final VoltageOut homeControl = new VoltageOut(0).withEnableFOC(false);
    private final TalonFX follower = ElevatorWristConstants.leftElevatorMotorConfig.createDevice(TalonFX::new);
    private final Follower followerControl = new Follower(leader.getDeviceID(), true);
    private final TalonFX wrist = ElevatorWristConstants.wristMotorConfig.createDevice(TalonFX::new);

    /* Sensors and Signals */
    private final Debouncer elevatorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
    private final StatusSignal<Angle> elevatorPositionStatus = leader.getPosition();
    // private final PositionTorqueCurrentFOC wristControl = new PositionTorqueCurrentFOC(ElevatorState.HOME.angle);
    private final StatusSignal<Current> elevatorCurrentStatus = leader.getStatorCurrent();
    private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
    private final Debouncer wristDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    private final CANcoder wristEncoder = ElevatorWristConstants.wristEncoderConfig.createDevice(CANcoder::new);
    private final StatusSignal<Angle> wristAngleStatus = wristEncoder.getPosition();

//    private final CANcoder homeCANcoder = ElevatorWristConstants.homeHallEffect.createDevice(CANcoder::new);
    private final VoltageOut tempVoltageControl = new VoltageOut(0).withEnableFOC(false);
    private final SysIdRoutine elevatorIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Units.Volts.of(4),
                    null,
                    state -> SignalLogger.writeString("SysIdElevatorState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    (volts) -> leader.setControl(new TorqueCurrentFOC(volts.in(Units.Volts))),
                    null,
                    this
            )
    );
    private final SysIdRoutine wristIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Units.Volts.of(0.25).per(Units.Seconds),
                    Units.Volts.of(1),
                    null,
                    state -> SignalLogger.writeString("SysIdWristState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    (volts) -> wrist.setControl(new TorqueCurrentFOC(volts.in(Units.Volts))),
                    null,
                    this
            )
    );

    private boolean elevatorStalled = false;

    /* State Machine Logic */
    private ElevatorState prevState = null;
    private ElevatorState state = ElevatorState.IDLE;
    private WristOrder wristOrder = WristOrder.MOVE_BOTH;
    private Distance requestedHeight = Distance.ofBaseUnits(-1, Units.Inches);
    private Angle requestedAngle = Angle.ofBaseUnits(-1, Units.Degrees);

    private boolean requestHome = false;
    private boolean requestIdle = false;
    private boolean requestChuteIntake = false;
    //    private boolean requestL1Score = false;
    private boolean requestL2Score = false;
    private boolean requestL3Score = false;
    //    private boolean requestL4Score = false;
    private boolean requestClimb = false;

    /* Other Variables */
    private boolean homedOnce = true;
    private boolean homing = false;
    private boolean elevatorAtPosition = false;
    private boolean wristAtPosition = false;
//    private final LightsSubsystem lightSubsystem = LightsSubsystem.getInstance();
//    private ElevatorWristSim sim = null;

    private static ElevatorWristSubsystem instance;
    public static ElevatorWristSubsystem getInstance() {
        if (instance == null) instance = new ElevatorWristSubsystem();
        return instance;
    }

    private ElevatorWristSubsystem() {
//        if (Utils.isSimulation()) sim = ElevatorWristSim.getInstance();

        // TODO: add configs for leader in ElevatorWristConstants
        // leader.setControl(leaderControl);

        // TODO: add configs for follower in ElevatorWristConstants
        follower.setControl(followerControl);

        // TODO: add configs for wrist in ElevatorWristConstants
        // wrist.setControl(wristControl);

        leader.setPosition(0);
    }

    public Command elevatorQuasistaticId(boolean forward) {
        return elevatorIdRoutine.quasistatic(forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
    }

    public Command elevatorDynamicId(boolean forward) {
        return elevatorIdRoutine.dynamic(forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
    }

    public Command wristQuasistaticId(boolean forward) {
        return wristIdRoutine.quasistatic(forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
    }

    public Command wristDynamicId(boolean forward) {
        return wristIdRoutine.dynamic(forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
    }

    private void homeElevator() {
        //Force the elevator to move down until the home switch is trigger at a slower speed for safety
        homeControl.withOutput(-0.8);
        homing = true;
        // leader.setControl(homeControl);
    }

    private void setElevatorHeight(Distance height) {
        // leaderControl.withPosition(Units.Revolutions.of(height.in(Units.Inches) * ElevatorWristConstants.revolutionsPerInch));
        // if (!homing) leader.setControl(leaderControl);
    }

    private void setWristAngle(Angle angle) {
        // wristControl.withPosition(angle.in(Revolutions));
        // wrist.setControl(wristControl);
    }

    @Override
    public void periodic() {
        ElevatorState nextState = getNextState();

        if (!homing) {
            state = nextState;
            unsetAllRequests();

            if (state == ElevatorState.HOME) homeElevator();
            else {
                if ((wristOrder != WristOrder.MOVE_FIRST || wristAtPosition) && // scuffed logic to make sure the elevator doesn't move before the wrist
                        !state.height.isEquivalent(requestedHeight)) { // buffer requests to hopefully reduce overhead
                    requestedHeight = state.height;
                    setElevatorHeight(state.height);
                }
            }
            if ((wristOrder != WristOrder.MOVE_LAST || elevatorAtPosition) && // scuffed logic to make sure the wrist doesn't move before the elevator
                    !state.angle.isEquivalent(requestedAngle)) { // buffer requests to hopefully reduce overhead
                requestedAngle = state.angle;
                setWristAngle(state.angle);
            }

//            lightSubsystem.requestColor(state.color); // TODO: enable lights
        } else {
            homingPeriodic(); // TODO: test homing
        }

        elevatorPositionStatus.refresh();
        elevatorCurrentStatus.refresh();
        wristAngleStatus.refresh();

        elevatorAtPosition = elevatorDebouncer.calculate(Math.abs(elevatorPositionStatus.getValueAsDouble() - state.height.magnitude()) < 10);
        wristAtPosition = wristDebouncer.calculate(Math.abs(wristAngleStatus.getValueAsDouble() - state.angle.magnitude()) < 5);
        elevatorStalled = currentFilter.calculate(elevatorCurrentStatus.getValueAsDouble()) > 20;


        SmartDashboard.putString("Elevator State", state.toString());
        SmartDashboard.putString("Prev Elevator State", prevState != null ? prevState.toString() : "null");

        SmartDashboard.putNumber("Elevator Height", elevatorPositionStatus.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Current", elevatorCurrentStatus.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Setpoint", state.height.in(Units.Inches) * ElevatorWristConstants.revolutionsPerInch);
        SmartDashboard.putNumber("Wrist Angle", wristAngleStatus.getValueAsDouble());
        SmartDashboard.putNumber("Wrist Setpoint", state.angle.in(Units.Revolutions));

        SmartDashboard.putBoolean("Homed Once", homedOnce);
        SmartDashboard.putBoolean("Elevator At Position", elevatorAtPosition);
        SmartDashboard.putBoolean("Wrist At Position", wristAtPosition);
        SmartDashboard.putBoolean("Home Switch", getHomeCANcoder());
        SmartDashboard.putBoolean("Both At Position", elevatorAtPosition && wristAtPosition);

        if (elevatorAtPosition && wristAtPosition) prevState = state;
    }

    private void homingPeriodic() {
//         if (getHomeCANcoder() || elevatorStalled) {
//             homing = false;
//             homedOnce = true;
//             leader.setPosition(0);
// //            System.out.println("At Home Position: " + (getHomeCANcoder() ? "Home Switch" : "Current"));
//         }
    }

    public void setRawVoltage(double rawInput) {
        tempVoltageControl.withOutput(rawInput * 2.5 + 0.38);
        //System.out.println("lets goo" + rawInput + 0.38);
        leader.setControl(tempVoltageControl);
    }

    private ElevatorState getNextState() {
        ElevatorState nextState = state;

        if (requestIdle) nextState = ElevatorState.IDLE;
        else if (requestChuteIntake) nextState = ElevatorState.CHUTE_INTAKE;
//        else if (requestL1Score) nextState = ElevatorState.L1_SCORE;
        else if (requestL2Score) nextState = ElevatorState.L2_SCORE;
        else if (requestL3Score) nextState = ElevatorState.L3_SCORE;
//        else if (requestL4Score) nextState = ElevatorState.L4_SCORE;
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

//    @Override
//    public void simulationPeriodic() {
//        sim.update(state.name(), state.height, state.angle);
//    }

    public void setBrakeMode() {
        leader.setNeutralMode(NeutralModeValue.Brake);
        follower.setNeutralMode(NeutralModeValue.Brake);

        wrist.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setCoastMode() {
        leader.setNeutralMode(NeutralModeValue.Coast);
        follower.setNeutralMode(NeutralModeValue.Coast);

        wrist.setNeutralMode(NeutralModeValue.Coast);
    }

    public double movePercent() {
        return Math.hypot(state.height.minus(prevState.height).magnitude(), state.angle.minus(prevState.angle).magnitude());
    }

    private void unsetAllRequests() {
        requestHome = false;
        requestIdle = false;
        requestChuteIntake = false;
//        requestL1Score = false;
        requestL2Score = false;
        requestL3Score = false;
//        requestL4Score = false;
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

//    public void requestL1Score(WristOrder order) {
//        unsetAllRequests();
//        requestL1Score = true;
//        wristOrder = order;
//    }

    public void requestClimb(WristOrder order) {
        unsetAllRequests();
        requestClimb = true;
        wristOrder = order;
    }

//    public void requestL4Score(WristOrder order) {
//        unsetAllRequests();
//        requestL4Score = true;
//        wristOrder = order;
//    }

}
