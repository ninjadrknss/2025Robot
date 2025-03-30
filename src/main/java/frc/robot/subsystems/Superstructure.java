package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem.WristOrder;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lights.LightsSubsystem;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance = null;

    enum SuperstructureState {
        PRE_HOME,
        IDLE,
        CHUTE_INTAKE,
//        L1_SCORE,
        L2_SCORE,
        L3_SCORE,
        L4_SCORE,
        CLIMB
    }

    /* Subsystems */
    private final ElevatorWristSubsystem elevatorWristSubsystem = ElevatorWristSubsystem.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final LightsSubsystem lightsSubsystem = LightsSubsystem.getInstance();

    /* State Flags */
    boolean requestHome = false;
    boolean requestIdle = true;
    boolean requestChuteIntake = false;
//    boolean requestL1Score = false;
    boolean requestL2Score = false;
    boolean requestL3Score = false;
    boolean requestL4Score = false;
    boolean requestClimb = false;

    /* Other Variables */
    private double mStateStartTime = 0.0;
    private SuperstructureState systemState = SuperstructureState.IDLE;

    boolean homedOnce = true;
    private double lastFPGATimestamp = 0.0;

    private Superstructure() {}

    public static Superstructure getInstance() {
        if (instance == null) instance = new Superstructure();
        return instance;
    }

    @Override
    public void periodic() {
        double time = RobotController.getFPGATime() / 1.0E6;
        Logger.recordOutput("Superstructure/loopCycleTime", time - lastFPGATimestamp);

        lastFPGATimestamp = time;
        SmartDashboard.putString("Superstructure/Superstructure State", systemState.toString());

        SuperstructureState nextState = systemState;
        switch (systemState) {
            case PRE_HOME -> {
                elevatorWristSubsystem.requestHome(WristOrder.MOVE_BOTH);
                if (elevatorWristSubsystem.homedOnce()) {
                    homedOnce = true;
                    nextState = SuperstructureState.IDLE;
                }
            }
            case IDLE -> {
                if (requestL2Score) {
                    elevatorWristSubsystem.requestL2Score(WristOrder.MOVE_FIRST);
                    nextState = SuperstructureState.L2_SCORE;
                } else if (requestL3Score) {
                    elevatorWristSubsystem.requestL3Score(WristOrder.MOVE_FIRST);
                    nextState = SuperstructureState.L3_SCORE;
                } else if (requestL4Score) {
                    elevatorWristSubsystem.requestL4Score(WristOrder.MOVE_FIRST);
                    nextState = SuperstructureState.L4_SCORE;
                } else if (requestClimb) {
                    elevatorWristSubsystem.requestClimb(WristOrder.MOVE_BOTH);
                    nextState = SuperstructureState.CLIMB;
                } else if (requestChuteIntake) {
                    elevatorWristSubsystem.requestChuteIntake(WristOrder.MOVE_BOTH);
                    nextState = SuperstructureState.CHUTE_INTAKE;
                }
            }
            case CHUTE_INTAKE -> {
                if (requestIdle) {
                    elevatorWristSubsystem.requestIdle(WristOrder.MOVE_BOTH);
                    nextState = SuperstructureState.IDLE;
                } else if (requestL2Score) {
                    elevatorWristSubsystem.requestL2Score(WristOrder.MOVE_FIRST);
                    nextState = SuperstructureState.L2_SCORE;
                } else if (requestL3Score) {
                    elevatorWristSubsystem.requestL3Score(WristOrder.MOVE_FIRST);
                    nextState = SuperstructureState.L3_SCORE;
                } else if (requestL4Score) {
                    elevatorWristSubsystem.requestL4Score(WristOrder.MOVE_FIRST);
                    nextState = SuperstructureState.L4_SCORE;
                } else if (requestClimb) {
                    elevatorWristSubsystem.requestClimb(WristOrder.MOVE_BOTH);
                    nextState = SuperstructureState.CLIMB;
                }
            }
            case L2_SCORE -> {
                if (requestIdle) {
                    elevatorWristSubsystem.requestIdle(WristOrder.MOVE_LAST);
                } else if (requestChuteIntake) {
                    elevatorWristSubsystem.requestIdle(WristOrder.MOVE_LAST);
                } else if (requestL3Score) {
                    elevatorWristSubsystem.requestL3Score(WristOrder.MOVE_BOTH);
                    nextState = SuperstructureState.L3_SCORE;
                } else if (requestL4Score) {
                    elevatorWristSubsystem.requestL4Score(WristOrder.MOVE_BOTH);
                    nextState = SuperstructureState.L4_SCORE;
                } else if (requestClimb) {
                    elevatorWristSubsystem.requestClimb(WristOrder.MOVE_LAST); // TODO: might just be move_both
                    nextState = SuperstructureState.CLIMB;
                }
            }

            case L3_SCORE -> {
                if (requestIdle) {
                    elevatorWristSubsystem.requestIdle(WristOrder.MOVE_LAST);
                } else if (requestChuteIntake) {
                    elevatorWristSubsystem.requestIdle(WristOrder.MOVE_LAST);
                } else if (requestL2Score) {
                    elevatorWristSubsystem.requestL2Score(WristOrder.MOVE_BOTH);
                    nextState = SuperstructureState.L2_SCORE;
                } else if (requestL4Score) {
                    elevatorWristSubsystem.requestL4Score(WristOrder.MOVE_BOTH);
                    nextState = SuperstructureState.L4_SCORE;
                } else if (requestClimb) {
                    elevatorWristSubsystem.requestClimb(WristOrder.MOVE_LAST); // TODO: might just be move_both
                    nextState = SuperstructureState.CLIMB;
                }
            }

            case L4_SCORE -> {
                if (requestIdle) {
                    elevatorWristSubsystem.requestIdle(WristOrder.MOVE_LAST);
                } else if (requestChuteIntake) {
                    elevatorWristSubsystem.requestIdle(WristOrder.MOVE_LAST);
                } else if (requestL2Score) {
                    elevatorWristSubsystem.requestL2Score(WristOrder.MOVE_BOTH);
                    nextState = SuperstructureState.L2_SCORE;
                } else if (requestL3Score) {
                    elevatorWristSubsystem.requestL4Score(WristOrder.MOVE_BOTH);
                    nextState = SuperstructureState.L3_SCORE;
                } else if (requestClimb) {
                    elevatorWristSubsystem.requestClimb(WristOrder.MOVE_LAST); // TODO: might just be move_both
                    nextState = SuperstructureState.CLIMB;
                }
            }

            case CLIMB -> {
                if (requestIdle) {
                    elevatorWristSubsystem.requestIdle(WristOrder.MOVE_BOTH);
                    nextState = SuperstructureState.IDLE;
                } else if (requestChuteIntake) {
                    elevatorWristSubsystem.requestChuteIntake(WristOrder.MOVE_BOTH);
                    nextState = SuperstructureState.CHUTE_INTAKE;
                } else if (requestL2Score) {
                    elevatorWristSubsystem.requestL2Score(WristOrder.MOVE_FIRST);
                    nextState = SuperstructureState.L2_SCORE;
                } else if (requestL3Score) {
                    elevatorWristSubsystem.requestL3Score(WristOrder.MOVE_FIRST);
                    nextState = SuperstructureState.L3_SCORE;
                } else if (requestL4Score) {
                    elevatorWristSubsystem.requestL4Score(WristOrder.MOVE_FIRST);
                    nextState = SuperstructureState.L4_SCORE;
                }
            }
            default -> throw new IllegalArgumentException("Guess I missed a state");
        }

        if (nextState != systemState) {
            mStateStartTime = time;
            systemState = nextState;
        }
    }

    public void unsetAllRequests() {
        requestHome = false;
        requestIdle = false;
        requestChuteIntake = false;
//        requestL1Score = false;
        requestL2Score = false;
        requestL3Score = false;
        requestL4Score = false;
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

//    public void requestL1Score() {
//        unsetAllRequests();
//        requestL1Score = true;
//    }

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

    public void requestClimb() {
        unsetAllRequests();
        requestClimb = true;
    }

    public boolean isAtPosition() {
         return elevatorWristSubsystem.isAtPosition();
    }
}
