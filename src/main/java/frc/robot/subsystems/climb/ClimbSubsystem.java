package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private static ClimbSubsystem instance;

    // Motor, sensor, and servo devices.
    private final TalonFX pivotMotor = ClimbConstants.pivotMotorConfig.createDevice(TalonFX::new);
    private final PositionTorqueCurrentFOC pivotControl = new PositionTorqueCurrentFOC(0)
            .withSlot(0);
    private final CANcoder pivotEncoder = ClimbConstants.pivotEncoderConfig.createDevice(CANcoder::new);
    private final Servo flapServo = new Servo(ClimbConstants.servoPort);

    // Define the states of the climber.
    public enum ClimbState {
        IDLE,
        STORE,
        DEPLOY,
        CONTROL
    }

    private ClimbState currentState = ClimbState.IDLE;

    // Target angles for pivot and flap (tunable via ClimbConstants).
    private boolean changes = true;
    private Angle targetPivotAngle = ClimbConstants.pivotStoreAngle;
    private Angle targetFlapAngle = ClimbConstants.flapStoreAngle;

    public static ClimbSubsystem getInstance() {
        if (instance == null) instance = new ClimbSubsystem();
        return instance;
    }
    
    private ClimbSubsystem() {
        pivotMotor.setControl(pivotControl);
    }

    public void requestStorePivot() {
        targetPivotAngle = ClimbConstants.pivotStoreAngle;
        currentState = ClimbState.STORE;
        changes = true;
    }

    public void requestDeployPivot() {
        targetPivotAngle = ClimbConstants.pivotDeployAngle;
        currentState = ClimbState.DEPLOY;
        changes = true;
    }

    public void requestStore(){
        requestStorePivot();
        requestStoreFlap();
    }

    public void requestStoreFlap() {
        targetFlapAngle = ClimbConstants.flapStoreAngle;
        currentState = ClimbState.STORE;
        changes = true;
    }

    public void requestDeployFlap() {
        targetFlapAngle = ClimbConstants.flapDeployAngle;
        currentState = ClimbState.DEPLOY;
        changes = true;
    }

    public void requestDeploy(){
        requestDeployPivot();
        requestDeployFlap();
    }
    
    public void increasePivotAngle() {
        modifyPivotAngle(ClimbConstants.changeRate);
        System.out.println(targetPivotAngle.in(Units.Degrees));
    }

    public void decreasePivotAngle() {
        modifyPivotAngle(ClimbConstants.changeRate.unaryMinus());
    }

    public void modifyPivotAngle(Angle delta) {
        targetPivotAngle = targetPivotAngle.plus(delta);
        currentState = ClimbState.CONTROL;
        changes = true;
    }

    @Override
    public void periodic() {
        // if (changes) {
            pivotControl.withPosition(targetPivotAngle);
            pivotMotor.setControl(pivotControl);
            flapServo.set(targetFlapAngle.in(Units.Rotations));
            changes = false;
        // }

        SmartDashboard.putString("Climb/Current State", currentState.name());
    }
}
