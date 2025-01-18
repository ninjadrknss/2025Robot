package frc.robot.util;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.commands.ChuteIntakeCommand;
import frc.robot.commands.GroundIntakeCommand;
import frc.robot.commands.HomeCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.SwerveConstants;

public class ControlBoard {
    private static ControlBoard instance;

    /* Controllers */
    private final PS5Controller driver = new PS5Controller(0);
//    private final PS5Controller operator = new PS5Controller(1);

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
            .withDeadband(SwerveConstants.maxSpeed * 0.005) // Add a 10% deadband
            .withRotationalDeadband(SwerveConstants.maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    private int level = 0;

    private ControlBoard() {
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

        configureDriverBindings();
        configureOperatorBindings();
    }

    public static ControlBoard getInstance() {
        if (instance == null) instance = new ControlBoard();
        return instance;
    }

    private void configureDriverBindings() {
        // TODO: configure driver bindings
    }

    private void configureOperatorBindings() {
        // TODO: configure operator bindings
    }

    public SwerveRequest getDriverRequest() {
        return driveRequest.withVelocityX(driver.leftVerticalJoystick.getAsDouble())
                .withVelocityY(driver.leftHorizontalJoystick.getAsDouble())
                .withRotationalRate(-driver.rightHorizontalJoystick.getAsDouble());
    }
}
