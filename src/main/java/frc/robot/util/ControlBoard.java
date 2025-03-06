package frc.robot.util;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.RunCommand;
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
    private final Superstructure superstructure;

    public GameElement desiredGoal;
    public GameElement previousConfirmedGoal;
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

        homeCommand = new HomeCommand(superstructure);
        chuteIntakeCommand = new ChuteIntakeCommand(superstructure);
        groundIntakeCommand = new GroundIntakeCommand(superstructure);
        L1ScoreCommand = new ScoreCommand(1);
        L2ScoreCommand = new ScoreCommand(2);
        L3ScoreCommand = new ScoreCommand(3);
        L4ScoreCommand = new ScoreCommand(4);
        BargeScoreCommand = new ScoreCommand(5);
        tryInit();
    }

    public void tryInit() {
        if (true || (driver == null && DriverStation.isJoystickConnected(ControllerPreset.DRIVER.port()))) {
            driver = new PS5Controller(ControllerPreset.DRIVER.port());
            configureBindings(ControllerPreset.DRIVER, driver);
            // Init operator bindings to driver:
            configureBindings(ControllerPreset.OPERATOR, driver);

            SwerveSubsystem drive = SwerveSubsystem.getInstance();
            drive.setDefaultCommand(drive.applyRequest(this::getDriverRequest));

            if (Utils.isSimulation()) drive.registerTelemetry(new MapSimSwerveTelemetry(SwerveConstants.maxSpeed)::telemeterize);
            System.out.println("Driver Initialized");
        }

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
        /* Driversim testing */
        //        controller.rightTrigger.onTrue(new InstantCommand(() ->
        //            SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(new Pose2d(1, 0.8, Rotation2d.fromDegrees(135)))))
        //            .ignoringDisable(true)
        //        );
        //        controller.leftBumper.onTrue(new InstantCommand(() ->
        //            SimulatedArena.getInstance().clearGamePieces())
        //            .ignoringDisable(true)
        //        );

        /* Driveassist testing */
//        controller.rightTrigger.whileTrue(new AssistCommand(superstructure, selectedBranch));

        /* Led Testing */
        //        LEDSubsystem led = LEDSubsystem.getInstance();
        //        controller.circleButton.onTrue(new InstantCommand(() -> led.requestColor(LEDSubsystem.Colors.MAGENTA)).ignoringDisable(true));
        //        controller.squareButton.onTrue(new InstantCommand(() -> led.requestColor(LEDSubsystem.Colors.WHITE)).ignoringDisable(true));
        //        controller.triangleButton.onTrue(new InstantCommand(() -> led.requestColor(LEDSubsystem.Colors.BLUE)).ignoringDisable(true));
        //        controller.crossButton.onTrue(new InstantCommand(() -> led.requestColor(LEDSubsystem.Colors.RED)).ignoringDisable(true));
        //
        //        controller.leftTrigger.onTrue(new InstantCommand(() -> led.requestRainbow()).ignoringDisable(true));
        //        controller.leftBumper.onTrue(new InstantCommand(() -> led.requestToggleBlinking()).ignoringDisable(true));

        /* Servo Testing */
        // Servo servoTest = new Servo(8);

        // controller.leftBumper.onTrue(new InstantCommand(() -> servoTest.setAngle(90)));
        // controller.rightBumper.onTrue(new InstantCommand(() -> servoTest.setAngle(0)));

        /* Elevator SysId */
        /*ElevatorWristSubsystem EWS = ElevatorWristSubsystem.getInstance();
        controller.leftBumper.whileTrue(EWS.elevatorDynamicId(true));
        controller.leftTrigger.whileTrue(EWS.elevatorDynamicId(false));
        controller.rightBumper.whileTrue(EWS.elevatorQuasistaticId(true));
        controller.rightTrigger.whileTrue(EWS.elevatorQuasistaticId(false));*/

        /* Wrist SysId */
        //        controller.leftBumper.whileTrue(EWS.wristDynamicId(true));
        //        controller.leftTrigger.whileTrue(EWS.wristDynamicId(false));
        //        controller.rightBumper.whileTrue(EWS.wristQuasistaticId(true));
        //        controller.rightTrigger.whileTrue(EWS.wristQuasistaticId(false));

        /* Climb SysId */
        ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();
//        controller.leftBumper.whileTrue(climbSubsystem.climberDynamicRoutine(true));
//        controller.leftTrigger.whileTrue(climbSubsystem.climberDynamicRoutine(false));
//        controller.rightBumper.whileTrue(climbSubsystem.climberQuasistaticRoutine(true));
//        controller.rightTrigger.whileTrue(climbSubsystem.climberQuasistaticRoutine(false));
        controller.rightTrigger.whileTrue(new InstantCommand(climbSubsystem::requestStore));
        controller.rightBumper.whileTrue(new InstantCommand(climbSubsystem::requestDeploy));
        controller.leftBumper.whileTrue(new RunCommand(climbSubsystem::increasePivotAngle));
        controller.leftTrigger.whileTrue(new RunCommand(climbSubsystem::decreasePivotAngle));

        controller.triangleButton.onTrue(new InstantCommand(SignalLogger::start).withName("Start Signal Logger"));
        controller.crossButton.onTrue(new InstantCommand(SignalLogger::stop).withName("Stop Signal Logger"));
        controller.squareButton.onTrue(new InstantCommand(() -> SwerveSubsystem.getInstance().resetPose(new Pose2d(3, 3, new Rotation2d(0)))).withName("Reset Pose"));
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
        double x = driver.leftVerticalJoystick.getAsDouble();
        double y = driver.leftHorizontalJoystick.getAsDouble();
        double rot = driver.rightHorizontalJoystick.getAsDouble();
        return driveRequest.withVelocityX(SwerveConstants.maxSpeed * x)
                .withVelocityY(SwerveConstants.maxSpeed * y)
                .withRotationalRate(SwerveConstants.maxAngularSpeed * Math.copySign(rot * rot, rot));
    }

    public String goalConfidence() {
        return String.format("%.0f%%", goalConfidence * 100);
    }
}
