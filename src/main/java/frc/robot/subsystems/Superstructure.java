package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance = null;

    enum SuperstructureState {
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
        Logger.recordOutput("Superstructure/loopCycleTime", Logger.getRealTimestamp()/1.0E6 - lastFPGATimestamp);

        lastFPGATimestamp = Logger.getRealTimestamp()/1.0E6;
        SmartDashboard.putString("Superstructure State", systemState.toString());

        SuperstructureState nextState = systemState;
        // TODO: Figure out how to handle climber stuff, most likely in ControlBoard.java
        switch (systemState) {
            case IDLE -> {}
            default -> throw new IllegalArgumentException("Guess I missed a state");
        }

        if (nextState != systemState) {
            mStateStartTime = Logger.getRealTimestamp() / 1.0E6;
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
