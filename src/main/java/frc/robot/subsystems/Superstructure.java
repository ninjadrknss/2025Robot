package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance = null;

    enum SuperstructureState { // TODO: did I miss some?
        PRE_HOME,
        IDLE,
        CHUTE_INTAKE,
        GROUND_INTAKE,
        L1_SCORE,
        L2_SCORE,
        L3_SCORE,
        L4_SCORE,
        CLIMB
    }

    /* Subsystems */
    private final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

    /* State Flags */
    boolean requestHome = true;

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
        // TODO: implement all states
        switch (systemState) {
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
    }

    public void requestHome() {
        unsetAllRequests();
        requestHome = true;
    }
}
