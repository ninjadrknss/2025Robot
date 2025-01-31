package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.SwerveSubsystem;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance = null;

    enum SuperstructureState { // TODO: ? add algae L2 and L3 Intake States
        PRE_HOME,
        IDLE,
        CHUTE_INTAKE,
        GROUND_INTAKE,
        L1_SCORE,
        L2_SCORE,
        L3_SCORE,
        L4_SCORE,
        BARGE_SCORE,
        CLIMB
    }

    /* Subsystems */
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
//     private final ElevatorWristSubsystem elevator = ElevatorWristSubsystem.getInstance();

    /* State Flags */
    boolean requestHome = true;
    boolean requestIdle = false;
    boolean requestChuteIntake = false;
    boolean requestGroundIntake = false;
    boolean requestL1Score = false;
    boolean requestL2Score = false;
    boolean requestL3Score = false;
    boolean requestL4Score = false;
    boolean requestBargeScore = false;
    boolean requestClimb = false;

    /* Other Variables */
    private double mStateStartTime = 0.0;
    private SuperstructureState systemState = SuperstructureState.PRE_HOME;

    boolean homedOnce = false;
    private double lastFPGATimestamp = 0.0;

    private Superstructure() {

    }

    public static Superstructure getInstance() {
        if (instance == null) instance = new Superstructure();
        return instance;
    }

    @Override
    public void periodic() {
        double time = RobotController.getFPGATime() / 1.0E6;
        Logger.recordOutput("Superstructure/loopCycleTime", time - lastFPGATimestamp);

        lastFPGATimestamp = time;
        SmartDashboard.putString("Superstructure State", systemState.toString());

        SuperstructureState nextState = systemState;
//        if (nextState == SuperstructureState.PRE_HOME && !homedOnce) {
//            elevator.requestHome();
//            if (elevator.isAtPosition()) nextState = SuperstructureState.IDLE;
//        }

        if (requestHome || requestIdle) {
            nextState = SuperstructureState.IDLE;
        }
        // TODO: implement all states
        switch (systemState) {
            case PRE_HOME -> {}
            case IDLE -> {}
            default -> throw new IllegalArgumentException("Guess I missed a state");
        }

        if (nextState != systemState) {
            mStateStartTime = RobotController.getFPGATime() / 1.0E6;
            systemState = nextState;
        }
    }

    public void unsetAllRequests() {
        requestHome = false;
        requestIdle = false;
        requestChuteIntake = false;
        requestGroundIntake = false;
        requestL1Score = false;
        requestL2Score = false;
        requestL3Score = false;
        requestL4Score = false;
        requestBargeScore = false;
        requestClimb = false;
    }

    public void requestHome() {
        unsetAllRequests();
        requestHome = true;
    }

    public void requestIdle() {
        unsetAllRequests();
        requestIdle = true;
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

    public void requestClimb() {
        unsetAllRequests();
        requestClimb = true;
    }

    public boolean isAtPosition() {
        // return elevator.isAtPosition();
        return false;
    }
}
