package frc.robot.util;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.PS5Controller;
import frc.robot.commands.AssistCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.LEDSubsystem.Color;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.simulation.Telemetry;

public class ControlBoard {
    private static ControlBoard instance;

    /* Controllers */
    private PS5Controller driver = null;
    private PS5Controller operator = null;

    /* Subsystems */
    private final Superstructure superstructure;

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

            Telemetry telemetry = new Telemetry(SwerveConstants.maxSpeed);
            SwerveSubsystem.getInstance().registerTelemetry(telemetry::telemeterize);

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
         driver.rightBumper.onTrue(new InstantCommand(() ->
             SwerveSubsystem.getInstance().resetPose(new Pose2d(3, 3, new Rotation2d())))
             .ignoringDisable(true)
         );
         driver.rightTrigger.onTrue(new InstantCommand(() ->
             SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(new Pose2d(1, 0.8, Rotation2d.fromDegrees(135)))))
             .ignoringDisable(true)
         );
         driver.leftBumper.onTrue(new InstantCommand(() ->
             SimulatedArena.getInstance().clearGamePieces())
             .ignoringDisable(true)
         );

        /* Led Testing */
//        LEDSubsystem led = LEDSubsystem.getInstance();
//        driver.circleButton.onTrue(new InstantCommand(() -> led.setColor(LEDSubsystem.Colors.magenta)).ignoringDisable(true));
//        driver.squareButton.onTrue(new InstantCommand(() -> led.setColor(LEDSubsystem.Colors.white)).ignoringDisable(true));
//        driver.triangleButton.onTrue(new InstantCommand(() -> led.setColor(LEDSubsystem.Colors.blue)).ignoringDisable(true));
//        driver.crossButton.onTrue(new InstantCommand(() -> led.setColor(LEDSubsystem.Colors.red)).ignoringDisable(true));
//
//        driver.leftTrigger.onTrue(new InstantCommand(() -> led.setRainbow()).ignoringDisable(true));

        // driver.rightTrigger.whileTrue(new AssistCommand(superstructure));
        
        // Servo servoTest = new Servo(8);

        // driver.leftBumper.onTrue(new InstantCommand(() -> servoTest.setAngle(90)));
        // driver.rightBumper.onTrue(new InstantCommand(() -> servoTest.setAngle(0)));
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
}
