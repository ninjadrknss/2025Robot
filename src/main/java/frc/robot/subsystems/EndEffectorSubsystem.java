package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {
    private static EndEffectorSubsystem instance;

    public static EndEffectorSubsystem getInstance() {
        if (instance == null) instance = new EndEffectorSubsystem();
        return instance;
    }

    private EndEffectorSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }
}

