package frc.robot.subsystems.endeffector;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {
    private static EndEffectorSubsystem instance;

    public static EndEffectorSubsystem getInstance() {
        if (instance == null) instance = new EndEffectorSubsystem();
        return instance;
    }

    private EndEffectorSubsystem() {
        // TODO: implement
        // Possible states? Spitting, Intaking, Idle
    }
}

