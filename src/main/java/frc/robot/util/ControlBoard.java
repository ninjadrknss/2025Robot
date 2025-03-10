package frc.robot.util;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.climb.ClimbSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.PS5Controller;

import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.subsystems.simulation.MapSimSwerveTelemetry;

import frc.robot.util.FieldConstants.GameElement;
import frc.robot.util.FieldConstants.GameElement.Branch;
import frc.robot.util.FieldConstants.GameElement.ScoreLevel;
import frc.robot.commands.ScoreCommand.*;

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
    private final Superstructure superstructure;

    public GameElement desiredGoal;
    public GameElement previousConfirmedGoal;
    public boolean preciseControl;
    public double goalConfidence;
    public Branch selectedBranch = Branch.LEFT;
    public ScoreLevel scoreLevel = ScoreLevel.L3;
    public boolean isAssisting = false;

    public GameElement prevDesiredGoal;


    /* Commands */
    private final HomeCommand homeCommand;
    private final ChuteIntakeCommand chuteIntakeCommand;
    private final GroundIntakeCommand groundIntakeCommand;
    private final ScoreCommand L1ScoreCommand;
    private final ScoreCommand L2ScoreCommand;
    private final ScoreCommand L3ScoreCommand;
    private final ScoreCommand L4ScoreCommand;
    private final ScoreCommand BargeScoreCommand;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.maxSpeed * 0.05) // Add a 10% deadband
            .withRotationalDeadband(SwerveConstants.maxAngularSpeed * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position)
            .withDesaturateWheelSpeeds(true);

    private ControlBoard() {
        DriverStation.silenceJoystickConnectionWarning(true); // TODO: remove
        superstructure = Superstructure.getInstance();

        desiredGoal = GameElement.PROCESSOR_BLUE;
        prevDesiredGoal = null;
        previousConfirmedGoal = null;
        preciseControl = false;

        homeCommand = new HomeCommand(superstructure);
        chuteIntakeCommand = new ChuteIntakeCommand(superstructure);
        groundIntakeCommand = new GroundIntakeCommand(superstructure);
        L1ScoreCommand = new ScoreCommand(Action.SCOREL1);
        L2ScoreCommand = new ScoreCommand(Action.SCOREL2);
        L3ScoreCommand = new ScoreCommand(Action.SCOREL3);
        L4ScoreCommand = new ScoreCommand(Action.SCOREL4);
        BargeScoreCommand = new ScoreCommand(Action.SCOREBARGE);
        tryInit();
    }

    public void tryInit() {
        driver = new PS5Controller(ControllerPreset.DRIVER.port());
        configureBindings(ControllerPreset.DRIVER, driver);

        SwerveSubsystem drive = SwerveSubsystem.getInstance();
        drive.setDefaultCommand(drive.applyRequest(this::getDriverRequest));

        if (Utils.isSimulation()) drive.registerTelemetry(new MapSimSwerveTelemetry(SwerveConstants.maxSpeed)::telemeterize);
        System.out.println("Driver Initialized");

        if (DriverStation.isJoystickConnected(ControllerPreset.OPERATOR.port()) && operator == null) {
            operator = new PS5Controller(ControllerPreset.OPERATOR.port());
            configureBindings(ControllerPreset.OPERATOR, operator);
            System.out.println("Operator Initialized");
        }
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
        // precise control (enabled (true) while leftTrigger is held. false as soon as it is released/while it is not held)
        controller.leftTrigger.whileTrue(new StartEndCommand(() -> preciseControl = true, () -> preciseControl = false).withName("Enable Precise Control"));
        controller.leftBumper.whileTrue(/* TODO: AIM ASSIST*/ new InstantCommand());

        /* Elevator SysId */
        ElevatorWristSubsystem EWS = ElevatorWristSubsystem.getInstance();
//        controller.leftBumper.whileTrue(EWS.elevatorDynamicId(true));
//        controller.leftTrigger.whileTrue(EWS.elevatorDynamicId(false));
//        controller.rightBumper.whileTrue(EWS.elevatorQuasistaticId(true));
//        controller.rightTrigger.whileTrue(EWS.elevatorQuasistaticId(false));

        /* Wrist SysId */
//        controller.leftBumper.whileTrue(EWS.wristDynamicId(true));
//        controller.leftTrigger.whileTrue(EWS.wristDynamicId(false));
//        controller.rightBumper.whileTrue(EWS.wristQuasistaticId(true));
//        controller.rightTrigger.whileTrue(EWS.wristQuasistaticId(false));
//        controller.circleButton.whileTrue(new InstantCommand(EWS::requestHome));
//        controller.squareButton.whileTrue(new InstantCommand(EWS::requestL2Score));

        /* Climb SysId */
//        ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();
//
//        controller.rightTrigger.whileTrue(new InstantCommand(climbSubsystem::requestStorePivot));
//        controller.rightBumper.whileTrue(new InstantCommand(climbSubsystem::requestDeployPivot));
//        controller.leftBumper.whileTrue(new RunCommand(climbSubsystem::increasePivotAngle));
//        controller.leftTrigger.whileTrue(new RunCommand(climbSubsystem::decreasePivotAngle));
//
//        controller.squareButton.whileTrue(new InstantCommand(climbSubsystem::requestDeployFlap));
//        controller.crossButton.whileTrue(new InstantCommand(climbSubsystem::requestStoreFlap));
        controller.triangleButton.onTrue(new InstantCommand(() -> SwerveSubsystem.getInstance().resetRotation(new Rotation2d(Math.PI))));

//        controller.triangleButton.onTrue(new InstantCommand(SignalLogger::start).withName("Start Signal Logger"));
//        controller.crossButton.onTrue(new InstantCommand(SignalLogger::stop).withName("Stop Signal Logger"));
    }

    private void configureOperatorBindings(PS5Controller controller) {
        controller.leftBumper.onTrue(new InstantCommand(() -> selectedBranch = Branch.LEFT).withName("Select Left Branch"));
        controller.touchpadButton.onTrue(new InstantCommand(() -> selectedBranch = Branch.CENTER).withName("Select Center Branch"));
        controller.rightBumper.onTrue(new InstantCommand(() -> selectedBranch = Branch.RIGHT).withName("Select Right Branch"));
        controller.triangleButton.onTrue(new InstantCommand(() -> { // SL 1 Up
            if(scoreLevel != ScoreLevel.L4) scoreLevel = ScoreLevel.values()[(scoreLevel.ordinal() + 1) % ScoreLevel.values().length];
        }).withName("Score Level Up"));
        controller.crossButton.onTrue(new InstantCommand(() -> { // SL 1 Down
            if(scoreLevel != ScoreLevel.L1) scoreLevel = ScoreLevel.values()[(scoreLevel.ordinal() - 1 + ScoreLevel.values().length) % ScoreLevel.values().length];
        }).withName("Score Level Down"));
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
