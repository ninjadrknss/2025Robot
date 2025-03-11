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
    private Angle targetPivotAngle = ClimbConstants.pivotStoreAngle;
    private Angle targetFlapAngle = ClimbConstants.flapStoreAngle;

    public static ClimbSubsystem getInstance() {
        if (instance == null) instance = new ClimbSubsystem();
        return instance;
    }
    
    private ClimbSubsystem() {
    }

    public void setTargetPivotAngle(Angle angle) {
        targetPivotAngle = angle;
        currentState = ClimbState.CONTROL;
    }

    public void setTargetFlapAngle(Angle angle) {
        targetFlapAngle = angle;
    }

    public void requestStorePivot() {
        targetPivotAngle = ClimbConstants.pivotStoreAngle;
        targetFlapAngle = ClimbConstants.flapStoreAngle;
        currentState = ClimbState.STORE;
    }

    public void requestDeployPivot() {
        targetPivotAngle = ClimbConstants.pivotDeployAngle;
        targetFlapAngle = ClimbConstants.flapDeployAngle;
        currentState = ClimbState.DEPLOY;
    }

    public void requestStore(){
        requestStorePivot();
        requestStoreFlap();
    }

    public void requestStoreFlap() {
        targetFlapAngle = ClimbConstants.flapStoreAngle;
        currentState = ClimbState.STORE;
    }

    public void requestDeployFlap() {
        targetFlapAngle = ClimbConstants.flapDeployAngle;
        currentState = ClimbState.DEPLOY;
    }

    public void requestDeploy(){
        requestDeployPivot();
        requestDeployFlap();
    }
    
    public void increasePivotAngle() {
        modifyPivotAngle(ClimbConstants.changeRate);
    }

    public void decreasePivotAngle() {
        modifyPivotAngle(ClimbConstants.changeRate.unaryMinus());
    }

    public void modifyPivotAngle(Angle delta) {
        targetPivotAngle = targetPivotAngle.plus(delta);
        currentState = ClimbState.CONTROL;
    }

    @Override
    public void periodic() {
        pivotControl.withPosition(targetPivotAngle);
        pivotMotor.setControl(pivotControl);
        flapServo.set(targetFlapAngle.in(Units.Rotations));

        SmartDashboard.putString("Climb/Current State", currentState.name());
    }
}
