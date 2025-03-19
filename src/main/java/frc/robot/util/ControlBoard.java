package frc.robot.util;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.climb.ClimbSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.PS5Controller;
import frc.robot.commands.ActionCommand.Action;

import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.simulation.MapSimSwerveTelemetry;
import frc.robot.util.FieldConstants.GameElement;
import frc.robot.util.FieldConstants.GameElement.Branch;
import frc.robot.util.FieldConstants.GameElement.ScoreLevel;

public class ControlBoard {
    private static ControlBoard instance;

    /* Controllers */
    private PS5Controller driver = null;
    private PS5Controller operator = null;

    private enum ControllerPreset {
        DRIVER(0),
        OPERATOR(1);

        private final int port;

        ControllerPreset(int port) {
            this.port = port;
        }

        public int port() {
            return port;
        }
    }

    /* Subsystems */
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final ElevatorWristSubsystem elevatorWristSubsystem = ElevatorWristSubsystem.getInstance();
    private final ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();

    /* State Variables */
    public GameElement desiredGoal;
    public GameElement previousConfirmedGoal;
    public boolean preciseControl;
    public double goalConfidence;
    public Branch selectedBranch = Branch.LEFT;
    public ScoreLevel scoreLevel = ScoreLevel.L3;
    public boolean isAssisting = false;

    public GameElement prevDesiredGoal;

    /* Commands */
    private final IdleCommand idleCommand;
    private final ScoreCommand scoreCommand;
    private final ChuteIntakeCommand chuteIntakeCommand;
//    private final ActionCommand L1ScoreCommand;
    private final ActionCommand L2ScoreCommand;
    private final ActionCommand L3ScoreCommand;
//    private final ActionCommand L4ScoreCommand;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.maxSpeed * 0.05) // Add a 5% deadband
            .withRotationalDeadband(SwerveConstants.maxAngularSpeed * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position)
            .withDesaturateWheelSpeeds(true);

    private ControlBoard() {
        DriverStation.silenceJoystickConnectionWarning(true); // TODO: remove

        desiredGoal = GameElement.PROCESSOR_BLUE;
        prevDesiredGoal = null;
        previousConfirmedGoal = null;
        preciseControl = false;

        idleCommand = new IdleCommand();
        chuteIntakeCommand = new ChuteIntakeCommand();
        scoreCommand = new ScoreCommand();
//        L1ScoreCommand = new ActionCommand(Action.SCOREL1);
        L2ScoreCommand = new ActionCommand(Action.SCOREL2);
        L3ScoreCommand = new ActionCommand(Action.SCOREL3);
//        L4ScoreCommand = new ActionCommand(Action.SCOREL4);
        tryInit();
    }

    public void tryInit() {
        if (driver == null) {
            driver = new PS5Controller(ControllerPreset.DRIVER.port());
            configureBindings(ControllerPreset.DRIVER, driver); // TODO: remove

            SwerveSubsystem drive = SwerveSubsystem.getInstance();
            drive.setDefaultCommand(drive.applyRequest(this::getDriverRequest));

            if (Utils.isSimulation())
                drive.registerTelemetry(new MapSimSwerveTelemetry(SwerveConstants.maxSpeed)::telemeterize);
            System.out.println("Driver Initialized");
        }

        if (/*DriverStation.isJoystickConnected(ControllerPreset.OPERATOR.port()) &&*/ operator == null) {
            operator = new PS5Controller(ControllerPreset.OPERATOR.port());
            configureBindings(ControllerPreset.OPERATOR, operator);
            System.out.println("Operator Initialized");
        }
    }

    public void displayUI(){
        SmartDashboard.putString("Current Goal", desiredGoal.name());
        SmartDashboard.putString("Current Level", scoreLevel.name());
        SmartDashboard.putString("Current Branch", selectedBranch.name());
    }

    public static ControlBoard getInstance() {
        if (instance == null) instance = new ControlBoard();
        return instance;
    }

    private void configureBindings(ControllerPreset preset, PS5Controller controller) {
        switch (preset) {
            case DRIVER -> configureDriverBindings(controller);
            case OPERATOR -> configureOperatorBindings(controller);
            default -> throw new IllegalStateException("Unexpected value: " + preset);
        }
    }

    private void configureDriverBindings(PS5Controller controller) {
        // Precise Control
        controller.rightTrigger.whileTrue(new StartEndCommand(() -> preciseControl = true, () -> preciseControl = false).withName("Precise Control Toggle")); // Fight me owen

        // Driver Assist
        // controller.rightBumper.whileTrue(new AssistCommand());

        // // Intake Subsystem
        // controller.leftTrigger.whileTrue(chuteIntakeCommand); // Run intakeSubsystem intaking, moving EWS to chute position
        // controller.leftBumper.whileTrue(scoreCommand); // Run intakeSubsystem spit, assume position handled already by operator

        controller.squareButton.whileTrue(new InstantCommand(() -> SwerveSubsystem.getInstance().resetRotation(SwerveSubsystem.getInstance().getOperatorForwardDirection())));
        // controller.triangleButton.whileTrue(new InstantCommand(elevatorWristSubsystem::requestL2Score));
        // controller.circleButton.whileTrue(new InstantCommand(elevatorWristSubsystem::requestIdle));

        controller.dUp.onTrue(new InstantCommand(climbSubsystem::requestDeployPivot));
        controller.dDown.onTrue(new InstantCommand(climbSubsystem::requestStorePivot));
        controller.touchpadButton.onTrue(new InstantCommand(climbSubsystem::requestClimbPivot));
        controller.dLeft.onTrue(new InstantCommand(climbSubsystem::requestDeployFlap));
        controller.dRight.onTrue(new InstantCommand(climbSubsystem::requestStoreFlap));

        // controller.dUp.whileTrue(swerveSubsystem.sysIdDynamic(Direction.kForward));
        // controller.dRight.whileTrue(swerveSubsystem.sysIdDynamic(Direction.kReverse));
        // controller.dDown.whileTrue(swerveSubsystem.sysIdQuasistatic(Direction.kForward));
        // controller.dLeft.whileTrue(swerveSubsystem.sysIdQuasistatic(Direction.kReverse));

    //    controller.triangleButton.onTrue(new InstantCommand(SignalLogger::start).withName("Start Signal Logger"));
    //    controller.crossButton.onTrue(new InstantCommand(SignalLogger::stop).withName("Stop Signal Logger"));
    }

    private void configureOperatorBindings(PS5Controller controller) {
        // Select Score Branch (LeftBumper, TouchpadClick, RightBumper)
        controller.leftBumper.onTrue(new InstantCommand(() -> selectedBranch = Branch.LEFT).withName("Select Left Branch"));
        controller.touchpadButton.onTrue(new InstantCommand(() -> selectedBranch = Branch.CENTER).withName("Select Center Branch"));
        controller.rightBumper.onTrue(new InstantCommand(() -> selectedBranch = Branch.RIGHT).withName("Select Right Branch"));
        // Select Score Level (TriangleButton, XButton)
        controller.triangleButton.onTrue(new InstantCommand(() -> { // SL 1 Up
            if(scoreLevel != ScoreLevel.L3) scoreLevel = ScoreLevel.values()[(scoreLevel.ordinal() + 1) % ScoreLevel.values().length];
        }).withName("Score Level Up"));
        controller.crossButton.onTrue(new InstantCommand(() -> { // SL 1 Down
            if(scoreLevel != ScoreLevel.L2) scoreLevel = ScoreLevel.values()[(scoreLevel.ordinal() - 1 + ScoreLevel.values().length) % ScoreLevel.values().length];
        }).withName("Score Level Down"));

        // Elevator Go To Selected Position (RightTrigger)
        controller.rightTrigger.onTrue(new ActionCommand(Action.PREPARE_SELECTED));

        ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();
        climbSubsystem.setDefaultCommand(new RunCommand(
                () -> climbSubsystem.setRawCurrent(controller.leftVerticalJoystick.getAsDouble()),
                climbSubsystem
        ));
    }


    public SwerveRequest getDriverRequest() {
        if (driver == null) return null;
        double scale = preciseControl ? 0.25 : 1.0;
        double rotScale = preciseControl ? 0.50 : 1.0;

        double x = driver.leftVerticalJoystick.getAsDouble() * scale;
        double y = driver.leftHorizontalJoystick.getAsDouble() * scale;
        double rot = driver.rightHorizontalJoystick.getAsDouble();
        return driveRequest.withVelocityX(SwerveConstants.maxSpeed * x)
                .withVelocityY(SwerveConstants.maxSpeed * y)
                .withRotationalRate(SwerveConstants.maxAngularSpeed * (Math.copySign(rot * rot, rot) * rotScale));
    }

    public String goalConfidence() {
        return String.format("%.0f%%", goalConfidence * 100);
    }
}
