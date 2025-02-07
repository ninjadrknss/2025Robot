package frc.robot.subsystems.simulation;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;

public class ElevatorWristSim {
    private static ElevatorWristSim instance;

    private static final double elevatorXPosition = 0;
    private static final double elevatorYPosition = 0;
    private static final double elevatorWidth = 0;
    private static final double elevatorHeight = 0;
    private static final double elevatorMaxHeight = 0.0;

    private static final double elevatorS1MinHeight = 0.0;
    private static final double elevatorS2MinHeight = 0.0;
    private static final double elevatorS3MinHeight = 0.0;

    private static final double elevatorS1MaxHeight = 0.0;
    private static final double elevatorS2MaxHeight = 0.0;
    private static final double elevatorS3MaxHeight = 0.0;

    private final Mechanism2d elevatorWrist = new Mechanism2d(elevatorWidth, elevatorHeight);
    private final MechanismRoot2d elevatorRoot;
    private final MechanismLigament2d elevatorS1 = new MechanismLigament2d("ElevS1", elevatorS1MinHeight, elevatorS1MaxHeight);
    private final MechanismLigament2d elevatorS2 = new MechanismLigament2d("ElevS2", elevatorS2MinHeight, elevatorS2MaxHeight);
    private final MechanismLigament2d elevatorS3 = new MechanismLigament2d("ElevS3", elevatorS3MinHeight, elevatorS3MaxHeight);

    private static final double wristMaxAngle = 0.0;
    private static final double wristMinAngle = 0.0;
    private final MechanismLigament2d wrist = new MechanismLigament2d("Wrist", wristMinAngle, wristMaxAngle);

    private final NetworkTable elevatorTable = NetworkTableInstance.getDefault().getTable("ElevatorWrist");
    private final Timer stateStart = new Timer();
    private double stateChangeTime = 0.0;

    private String currentState = "";

    public static ElevatorWristSim getInstance() {
        if (instance == null) instance = new ElevatorWristSim();
        return instance;
    }

    private ElevatorWristSim() {
        elevatorRoot = elevatorWrist.getRoot("Elevator", elevatorXPosition, elevatorYPosition);
        elevatorRoot.append(elevatorS1);
        elevatorS1.append(elevatorS2);
        elevatorS2.append(elevatorS3);
        elevatorS3.append(wrist);
    }

    public void update(String name, double height, double angle) {
        if (!name.equals(currentState)) {
            currentState = name;
            elevatorTable.getEntry("State").setString(name);
            stateStart.reset();
            stateChangeTime = stateStart.get();
        }
        double time = stateStart.get() - stateChangeTime;
        SmartDashboard.putData("ElevatorWrist", elevatorWrist);
    }
}
