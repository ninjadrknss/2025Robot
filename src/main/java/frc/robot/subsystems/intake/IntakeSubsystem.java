package frc.robot.subsystems.intake;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;

    public static IntakeSubsystem getInstance() {
        if (instance == null) instance = new IntakeSubsystem();
        return instance;
    }

    private IntakeSubsystem() {
        // TODO: implement
        // Possible states? Spitting, Intaking, Idle
    }
}

