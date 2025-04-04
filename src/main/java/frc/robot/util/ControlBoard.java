package frc.robot.util;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climb.ClimbSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.PS5Controller;

import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
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
    private final Superstructure superstructure = Superstructure.getInstance();
    private final ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();

    /* State Variables */
    public GameElement desiredGoal = GameElement.PROCESSOR_BLUE;
    public GameElement previousConfirmedGoal = null;
    public boolean preciseControl = false;
    public double goalConfidence;
    public Branch selectedBranch = Branch.LEFT;
    public ScoreLevel scoreLevel = ScoreLevel.L3;
    public boolean isAssisting = false;

    public GameElement prevDesiredGoal = null;

    /* Commands */
    private final IdleCommand idleCommand = new IdleCommand();
    private final SpitCommand spitCommand = new SpitCommand();
    private final IntakeCommand intakeCommand = new IntakeCommand();

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.maxSpeed * 0.05) // Add a 5% deadband
            .withRotationalDeadband(SwerveConstants.maxAngularSpeed * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position)
            .withDesaturateWheelSpeeds(true);

    private ControlBoard() {
        DriverStation.silenceJoystickConnectionWarning(true);

        tryInit();
    }

    public void tryInit() {
        if (driver == null) {
            driver = new PS5Controller(ControllerPreset.DRIVER.port());
            configureBindings(ControllerPreset.DRIVER, driver);

            SwerveSubsystem drive = SwerveSubsystem.getInstance();
            drive.setDefaultCommand(drive.applyRequest(this::getDriverRequest));
            if (Utils.isSimulation())
                drive.registerTelemetry(new MapSimSwerveTelemetry(SwerveConstants.maxSpeed)::telemeterize);
            System.out.println("Driver Initialized");
        }

        if (operator == null) {
            operator = new PS5Controller(ControllerPreset.OPERATOR.port());
            configureBindings(ControllerPreset.OPERATOR, operator);
            System.out.println("Operator Initialized");
        }
    }

    public void displayUI(){
        SmartDashboard.putBoolean("ControlBoard/preciseControl", preciseControl);
        SmartDashboard.putString("ControlBoard/Current Goal", desiredGoal.name());
        SmartDashboard.putString("ControlBoard/Current Level", scoreLevel.name());
        SmartDashboard.putNumber("ControlBoard/Score Level", scoreLevel.ordinal() + 2);
        SmartDashboard.putString("ControlBoard/Current Branch", selectedBranch.name());
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
        /* Precise Control */
         controller.rightBumper.whileTrue(new StartEndCommand(() -> preciseControl = true, () -> preciseControl = false).withName("Precise Control Toggle")); // Fight me owen

        /* Driver Assist */
        controller.rightTrigger.whileTrue(new AssistCommand());

        /* Intake Subsystem */
        controller.leftTrigger.whileTrue(intakeCommand); // Run intakeSubsystem intaking, moving EWS to chute position
        controller.leftBumper.whileTrue(spitCommand); // Run intakeSubsystem spit, assume position handled already by operator

        controller.squareButton.whileTrue(new InstantCommand(() -> SwerveSubsystem.getInstance().resetRotation(SwerveSubsystem.getInstance().getOperatorForwardDirection())));
        controller.triangleButton.whileTrue(new InstantCommand(superstructure::requestL2Score).withName("L2 Score"));
        controller.circleButton.whileTrue(new InstantCommand(superstructure::requestL3Score).withName("L3 Score"));
        controller.crossButton.whileTrue(new InstantCommand(superstructure::requestL4Score).withName("L4 Score"));

        /* Climb Subsystem */
        controller.touchpadButton.whileTrue(new InstantCommand(superstructure::requestClimb).withName("Climb Elevator Command"));
        controller.dRight.whileTrue(new InstantCommand(climbSubsystem::requestDeployFlap));
        controller.dLeft.whileTrue(new InstantCommand(climbSubsystem::requestStoreFlap));
        controller.dUp.whileTrue(new InstantCommand(climbSubsystem::requestRatchetActive));
        controller.dDown.whileTrue(new InstantCommand(climbSubsystem::requestRatchetInActive));

//        controller.rightBumper.onTrue(new InstantCommand(SignalLogger::start).withName("Start Signal Logger"));
//        controller.rightTrigger.onTrue(new InstantCommand(SignalLogger::stop).withName("Stop Signal Logger"));
    }

    private void configureOperatorBindings(PS5Controller controller) {
        // Select Score Branch (LeftBumper, TouchpadClick, RightBumper)
        controller.leftBumper.onTrue(new InstantCommand(() -> selectedBranch = Branch.LEFT).withName("Select Left Branch"));
        controller.touchpadButton.onTrue(new InstantCommand(() -> selectedBranch = Branch.CENTER).withName("Select Center Branch"));
        controller.rightBumper.onTrue(new InstantCommand(() -> selectedBranch = Branch.RIGHT).withName("Select Right Branch"));

        // Select Score Level (TriangleButton, XButton)
        controller.squareButton.whileTrue(new InstantCommand(superstructure::requestHome).withName("Home Elevator Command"));
        controller.triangleButton.onTrue(new InstantCommand(() -> scoreLevel = ScoreLevel.L2).withName("L2 Score Select"));
        controller.circleButton.onTrue(new InstantCommand(() -> scoreLevel = ScoreLevel.L3).withName("L3 Score Select"));
        controller.crossButton.onTrue(new InstantCommand(() -> scoreLevel = ScoreLevel.L4).withName("L4 Score Select"));

        controller.dUp.whileTrue(new InstantCommand(climbSubsystem::requestRatchetActive));
        controller.dDown.whileTrue(new InstantCommand(climbSubsystem::requestRatchetInActive));
        controller.dLeft.onTrue(new InstantCommand(superstructure::requestClimb).withName("Climb Elevator Command"));

        // Elevator Go To Selected Position (RightTrigger)
        controller.rightTrigger.whileTrue(new ElevatorWristCommand()); // Go to selected position while held, on release go to idle
    }

    public double getOperatorLeftVertical() { // used for jank climber open loop control
        return operator == null ? 0 : operator.leftVerticalJoystick.getAsDouble();
    }

    public SwerveRequest getDriverRequest() {
        if (driver == null) return null;

        boolean tippyMode = ElevatorWristSubsystem.getInstance().isTall();
        double scale = preciseControl || tippyMode ? 0.25 : 1.0;
        double rotScale = preciseControl || tippyMode ? 0.50 : 1.0;

        double x = driver.leftVerticalJoystick.getAsDouble();
        double y = driver.leftHorizontalJoystick.getAsDouble();
        double rot = driver.rightHorizontalJoystick.getAsDouble();
        return driveRequest.withVelocityX(SwerveConstants.maxSpeed * x * scale)
                           .withVelocityY(SwerveConstants.maxSpeed * y * scale)
                           .withRotationalRate(SwerveConstants.maxAngularSpeed * (Math.copySign(rot * rot, rot) * rotScale));
    }

    public String goalConfidence() {
        return String.format("%.0f%%", goalConfidence * 100);
    }
}
