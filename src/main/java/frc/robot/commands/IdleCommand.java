package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class IdleCommand extends Command {
  private final Superstructure superstructure = Superstructure.getInstance();

  public IdleCommand() {
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.requestIdle();
  }
}
