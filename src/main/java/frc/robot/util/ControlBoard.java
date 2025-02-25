package frc.robot.util;

import com.ctre.phoenix6.SignalLogger;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.PS5Controller;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.simulation.MapSimSwerveTelemetry;
import frc.robot.commands.AssistCommand; 

public class ControlBoard {
    private static ControlBoard instance;

    /* Controllers */
    private PS5Controller driver = null;
    private PS5Controller operator = null;

    /* Subsystems */
    private final Superstructure superstructure;

    public Constants.GameElement desiredGoal;
    public Constants.GameElement previousConfirmedGoal;
    public double goalConfidence;
    public Constants.GameElement.Branch selectedBranch = Constants.GameElement.Branch.LEFT;
    public Constants.GameElement.ScoreLevel scoreLevel = Constants.GameElement.ScoreLevel.L3;

    public Constants.GameElement prevDesiredGoal;

    /* Commands */
//    private final HomeCommand homeCommand;
//    private final ChuteIntakeCommand chuteIntakeCommand;
//    private final GroundIntakeCommand groundIntakeCommand;
//    private final ScoreCommand L1ScoreCommand;
//    private final ScoreCommand L2ScoreCommand;
//    private final ScoreCommand L3ScoreCommand;
//    private final ScoreCommand L4ScoreCommand;
//    private final ScoreCommand BargeScoreCommand;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.maxSpeed * 0.05) // Add a 10% deadband
            .withRotationalDeadband(SwerveConstants.maxAngularSpeed * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    private int level = 0;

    private ControlBoard() {
        DriverStation.silenceJoystickConnectionWarning(true); // TODO: remove
        superstructure = Superstructure.getInstance();

        desiredGoal = Constants.GameElement.PROCESSOR_BLUE;
        prevDesiredGoal = null;
        previousConfirmedGoal = null;


//        homeCommand = new HomeCommand(superstructure);
//
//        chuteIntakeCommand = new ChuteIntakeCommand(superstructure);
//        groundIntakeCommand = new GroundIntakeCommand(superstructure);
//
//        L1ScoreCommand = new ScoreCommand(superstructure, 1);
//        L2ScoreCommand = new ScoreCommand(superstructure, 2);
//        L3ScoreCommand = new ScoreCommand(superstructure, 3);
//        L4ScoreCommand = new ScoreCommand(superstructure, 4);
//        BargeScoreCommand = new ScoreCommand(superstructure, 5);
        tryInit();
    }

    public void tryInit() {
        // if (DriverStation.isJoystickConnected(0) && driver == null) {
            driver = new PS5Controller(0);
            System.out.println("Driver Configured");
            configureDriverBindings();

            SwerveSubsystem.getInstance().registerTelemetry(new MapSimSwerveTelemetry(SwerveConstants.maxSpeed)::telemeterize);

            SwerveSubsystem drivetrain = SwerveSubsystem.getInstance();
            drivetrain.setDefaultCommand(
                drivetrain.applyRequest(this::getDriverRequest)
            );
        // }
        // if (DriverStation.isJoystickConnected(1) && operator == null) {
            // operator = new PS5Controller(1);
            // configureOperatorBindings();
        // }
    }

    public static ControlBoard getInstance() {
        if (instance == null) instance = new ControlBoard();
        return instance;
    }

    private void configureDriverBindings() {
        // TODO: configure driver bindings
        /* Driversim testing */
//        driver.rightBumper.onTrue(new InstantCommand(() ->
//            SwerveSubsystem.getInstance().resetPose(new Pose2d(3, 3, new Rotation2d())))
//            .ignoringDisable(true)
//        );
//        driver.rightTrigger.onTrue(new InstantCommand(() ->
//            SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(new Pose2d(1, 0.8, Rotation2d.fromDegrees(135)))))
//            .ignoringDisable(true)
//        );
//        driver.leftBumper.onTrue(new InstantCommand(() ->
//            SimulatedArena.getInstance().clearGamePieces())
//            .ignoringDisable(true)
//        );

        /* Driveassist testing */
       driver.leftBumper.whileTrue(new AssistCommand(superstructure, Constants.GameElement.Branch.LEFT));
       driver.rightBumper.whileTrue(new AssistCommand(superstructure, Constants.GameElement.Branch.RIGHT));
//        driver.leftTrigger.onTrue(new InstantCommand(
//                () -> SwerveSubsystem.getInstance().resetPose(new Pose2d(3, 3, new Rotation2d()))
//        ).ignoringDisable(true));

        /* Led Testing */
//        LEDSubsystem led = LEDSubsystem.getInstance();
//        driver.circleButton.onTrue(new InstantCommand(() -> led.requestColor(LEDSubsystem.Colors.MAGENTA)).ignoringDisable(true));
//        driver.squareButton.onTrue(new InstantCommand(() -> led.requestColor(LEDSubsystem.Colors.WHITE)).ignoringDisable(true));
//        driver.triangleButton.onTrue(new InstantCommand(() -> led.requestColor(LEDSubsystem.Colors.BLUE)).ignoringDisable(true));
//        driver.crossButton.onTrue(new InstantCommand(() -> led.requestColor(LEDSubsystem.Colors.RED)).ignoringDisable(true));
//
//        driver.leftTrigger.onTrue(new InstantCommand(() -> led.requestRainbow()).ignoringDisable(true));
//        driver.leftBumper.onTrue(new InstantCommand(() -> led.requestToggleBlinking()).ignoringDisable(true));

        /* Servo Testing */
        // Servo servoTest = new Servo(8);

        // driver.leftBumper.onTrue(new InstantCommand(() -> servoTest.setAngle(90)));
        // driver.rightBumper.onTrue(new InstantCommand(() -> servoTest.setAngle(0)));

        /* Elevator SysId */
        // ElevatorWristSubsystem EWS = ElevatorWristSubsystem.getInstance();
        // driver.leftBumper.whileTrue(EWS.elevatorDynamicId(true));
        // driver.leftTrigger.whileTrue(EWS.elevatorDynamicId(false));
        // driver.rightBumper.whileTrue(EWS.elevatorQuasistaticId(true));
        // driver.rightTrigger.whileTrue(EWS.elevatorQuasistaticId(false));

        /* Wrist SysId */
//        driver.leftBumper.whileTrue(EWS.wristDynamicId(true));
//        driver.leftTrigger.whileTrue(EWS.wristDynamicId(false));
//        driver.rightBumper.whileTrue(EWS.wristQuasistaticId(true));
//        driver.rightTrigger.whileTrue(EWS.wristQuasistaticId(false));

        driver.triangleButton.onTrue(new InstantCommand(SignalLogger::start));
        driver.crossButton.onTrue(new InstantCommand(SignalLogger::stop));
    }

    private void configureOperatorBindings() {
        // TODO: configure operator bindings
    }

    public SwerveRequest getDriverRequest() {
        if (driver == null) return null;
        double x = -driver.leftVerticalJoystick.getAsDouble();
        double y = -driver.leftHorizontalJoystick.getAsDouble();
        double rot = driver.rightHorizontalJoystick.getAsDouble();
        return driveRequest.withVelocityX(SwerveConstants.maxSpeed * x)
                .withVelocityY(SwerveConstants.maxSpeed * y)
                .withRotationalRate(SwerveConstants.maxAngularSpeed * Math.copySign(rot * rot, rot));
    }

    public String goalConfidence() {
        return String.format("%.0f%%", goalConfidence * 100);
    }
}
