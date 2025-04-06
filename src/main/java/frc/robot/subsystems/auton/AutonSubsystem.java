package frc.robot.subsystems.auton;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.subsystems.drive.Odometry;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AssistCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.FieldConstants.GameElement;
// import the wpilib waitcommand
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonSubsystem {
    private final AutoChooser autoChooser = new AutoChooser();

    private final AutoFactory autoFactory;

    private final Superstructure superstructure;
    private final SwerveSubsystem swerveSubsystem;

    private static AutonSubsystem instance;

    private AutonSubsystem() {
        superstructure = Superstructure.getInstance();
        swerveSubsystem = SwerveSubsystem.getInstance();

        autoFactory = new AutoFactory(
            swerveSubsystem::getPose,
            swerveSubsystem::resetPose,
            swerveSubsystem::followPath,
            false,
            swerveSubsystem
        );
       
        autoChooser.addRoutine("hardcode auton", () -> getBadAuton());
        autoChooser.addRoutine("BlueRight", () -> getBlueRight());
        autoChooser.addRoutine("BlueMid", () -> getBlueMid());
        autoChooser.addRoutine("BlueLeft", () -> getBlueLeft());
        autoChooser.addRoutine("RedRight", () -> getRedRight());
        autoChooser.addRoutine("RedMid", () -> getRedMid());
        autoChooser.addRoutine("RedLeft", () -> getRedLeft());
        autoChooser.addRoutine("MoveAuton", () -> getMoveAuton());
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public static AutonSubsystem getInstance() {
        if (instance == null) instance = new AutonSubsystem();
        return instance;
    }

    public Command getSelectedAuton() {
        return autoChooser.selectedCommand();
    }

    private AutoRoutine getAuton(String name) {
        AutoRoutine routine = autoFactory.newRoutine(name);
        List<Command> commandList = new ArrayList<>();
        
        int index = 0;
        while (true) {
            AutoTrajectory trajectory = routine.trajectory(name, index);
            if (trajectory.getFinalPose().equals(Optional.empty())) break;
            if (index == 0) commandList.add(trajectory.resetOdometry());
            commandList.add(trajectory.cmd());
            
            //commandList.add(new AssistCommand(false, true));
            //commandList.add(new WaitCommand(1));
            index++;
        }

        // Register the full sequence of commands to run when routine is active
        routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
        return routine;
    }

    private AutoRoutine getBlueRight(){
        AutoRoutine routine = autoFactory.newRoutine("blueright");
        List<Command> commandList = new ArrayList<>();
        
        commandList.add(new AssistCommand(GameElement.REEF_BLUE_2, GameElement.Branch.LEFT));
        commandList.add(Commands.race(
                new ScoreCommand(ScoreCommand.Level.L4),
                new WaitCommand(5)
        ));

        commandList.add(new AssistCommand(GameElement.CORAL_STATION_BLUE_1, null));
        commandList.add(Commands.race(
                new IntakeCommand(),
                new WaitCommand(10)
        ));
        //commandList.add(new WaitCommand(5));

        commandList.add(new AssistCommand(GameElement.REEF_BLUE_4, GameElement.Branch.RIGHT));
        commandList.add(Commands.race(
                new ScoreCommand(ScoreCommand.Level.L4),
                new WaitCommand(5)
        ));

        //commandList.add(new AssistCommand(GameElement.CORAL_STATION_BLUE_1, null));
//        commandList.add(Commands.race(
//                new IntakeCommand(),
//                new WaitCommand(10)
//        ));

        //commandList.add(new AssistCommand(GameElement.REEF_RED_1, GameElement.Branch.LEFT));
        // Register the full sequence of commands to run when routine is active
        routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
        return routine; 
    }

    private AutoRoutine getBlueMid(){
        AutoRoutine routine = autoFactory.newRoutine("bluemid");
        List<Command> commandList = new ArrayList<>();
        
        commandList.add(new AssistCommand(GameElement.REEF_BLUE_1, GameElement.Branch.LEFT));
        commandList.add(Commands.race(
                new ScoreCommand(ScoreCommand.Level.L4),
                new WaitCommand(5)
        ));

        commandList.add(new AssistCommand(GameElement.CORAL_STATION_BLUE_1, null));
        commandList.add(Commands.race(
                new IntakeCommand(),
                new WaitCommand(10)
        ));

        commandList.add(new AssistCommand(GameElement.REEF_BLUE_6, GameElement.Branch.RIGHT));
        commandList.add(Commands.race(
                new ScoreCommand(ScoreCommand.Level.L4),
                new WaitCommand(5)
        ));
        routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
        return routine; 
    }

    private AutoRoutine getBlueLeft(){
        AutoRoutine routine = autoFactory.newRoutine("blueleft");
        List<Command> commandList = new ArrayList<>();
        
        commandList.add(new AssistCommand(GameElement.REEF_BLUE_3, GameElement.Branch.RIGHT));
        commandList.add(Commands.race(
                new ScoreCommand(ScoreCommand.Level.L4),
                new WaitCommand(5)
        ));

        commandList.add(new AssistCommand(GameElement.CORAL_STATION_BLUE_2, null));
        commandList.add(Commands.race(
                new IntakeCommand(),
                new WaitCommand(10)
        ));

        commandList.add(new AssistCommand(GameElement.REEF_BLUE_5, GameElement.Branch.RIGHT));
        commandList.add(Commands.race(
                new ScoreCommand(ScoreCommand.Level.L4),
                new WaitCommand(5)
        ));
        routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
        return routine; 
    }


    private AutoRoutine getRedRight(){
        AutoRoutine routine = autoFactory.newRoutine("redright");
        List<Command> commandList = new ArrayList<>();
        
        commandList.add(new AssistCommand(GameElement.REEF_RED_5, GameElement.Branch.LEFT));
        commandList.add(Commands.race(
                new ScoreCommand(ScoreCommand.Level.L4),
                new WaitCommand(5)
        ));

        commandList.add(new AssistCommand(GameElement.CORAL_STATION_RED_2, null));
        commandList.add(Commands.race(
                new IntakeCommand(),
                new WaitCommand(10)
        ));

        commandList.add(new AssistCommand(GameElement.REEF_RED_3, GameElement.Branch.RIGHT));
        commandList.add(Commands.race(
                new ScoreCommand(ScoreCommand.Level.L4),
                new WaitCommand(5)
        ));
        routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
        return routine; 
    }

    private AutoRoutine getRedMid(){
        AutoRoutine routine = autoFactory.newRoutine("redmid");
        List<Command> commandList = new ArrayList<>();
        
        commandList.add(new AssistCommand(GameElement.REEF_RED_6, GameElement.Branch.LEFT));
        commandList.add(Commands.race(
                new ScoreCommand(ScoreCommand.Level.L4),
                new WaitCommand(5)
        ));

        commandList.add(new AssistCommand(GameElement.CORAL_STATION_RED_2, null));
        commandList.add(Commands.race(
                new IntakeCommand(),
                new WaitCommand(10)
        ));

        commandList.add(new AssistCommand(GameElement.REEF_RED_1, GameElement.Branch.RIGHT));
        commandList.add(Commands.race(
                new ScoreCommand(ScoreCommand.Level.L4),
                new WaitCommand(5)
        ));
        routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
        return routine; 
    }

    private AutoRoutine getRedLeft(){
        AutoRoutine routine = autoFactory.newRoutine("redleft");
        List<Command> commandList = new ArrayList<>();
        
        commandList.add(new AssistCommand(GameElement.REEF_RED_4, GameElement.Branch.RIGHT));
        commandList.add(Commands.race(
                new ScoreCommand(ScoreCommand.Level.L4),
                new WaitCommand(5)
        ));

        commandList.add(new AssistCommand(GameElement.CORAL_STATION_RED_1, null));
        commandList.add(Commands.race(
                new IntakeCommand(),
                new WaitCommand(10)
        ));

        commandList.add(new AssistCommand(GameElement.REEF_RED_2, GameElement.Branch.RIGHT));
        commandList.add(Commands.race(
                new ScoreCommand(ScoreCommand.Level.L4),
                new WaitCommand(5)
        ));
        routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
        return routine; 
    }

    private AutoRoutine getMoveAuton() {
        AutoRoutine routine = autoFactory.newRoutine("MoveAuton");
        List<Command> commandList = new ArrayList<>();

        SwerveRequest swerveRequest = new SwerveRequest.FieldCentric().withVelocityX(-2).withVelocityY(0);

        commandList.add(new InstantCommand(() -> SwerveSubsystem.getInstance().resetRotation(SwerveSubsystem.getInstance().getOperatorForwardDirection())));
        commandList.add(SwerveSubsystem.getInstance().applyRequest(() -> swerveRequest));

        routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
        return routine;
    }


    private AutoRoutine getBadAuton(){
        AutoRoutine routine = autoFactory.newRoutine("imbad");
        List<Command> commandList = new ArrayList<>();
        
        commandList.add(new AssistCommand(GameElement.REEF_RED_6, GameElement.Branch.LEFT));
        commandList.add(Commands.race(
                new ScoreCommand(ScoreCommand.Level.L4),
                new WaitCommand(1)
        ));

        commandList.add(new AssistCommand(GameElement.CORAL_STATION_RED_1, null));
        commandList.add(Commands.race(
                new IntakeCommand(),
                new WaitCommand(1)
        ));

        commandList.add(new AssistCommand(GameElement.REEF_RED_3, GameElement.Branch.RIGHT));
        commandList.add(Commands.race(
                new ScoreCommand(ScoreCommand.Level.L4),
                new WaitCommand(1)
        ));

        commandList.add(new AssistCommand(GameElement.CORAL_STATION_RED_2, null));
        commandList.add(Commands.race(
                new IntakeCommand(),
                new WaitCommand(1)
        ));

        commandList.add(new AssistCommand(GameElement.REEF_RED_2, GameElement.Branch.LEFT));
        commandList.add(Commands.race(
                new ScoreCommand(ScoreCommand.Level.L4),
                new WaitCommand(1)
        ));

        commandList.add(new AssistCommand(GameElement.CORAL_STATION_RED_2, null));
        commandList.add(Commands.race(
                new IntakeCommand(),
                new WaitCommand(1)
        ));

        commandList.add(new AssistCommand(GameElement.REEF_RED_1, GameElement.Branch.LEFT));

        // Register the full sequence of commands to run when routine is active
        routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
        return routine;
    }

    private AutoRoutine getExampleAuton() {
        AutoRoutine routine = autoFactory.newRoutine("ExampleAuton");
        AutoTrajectory trajectory = routine.trajectory("ExampleAuton");

        routine.active().onTrue(
            trajectory.resetOdometry().andThen(
                trajectory.cmd()
            )
        );
        return routine;
    }
}
