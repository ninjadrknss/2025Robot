package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Revolutions;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ControlBoard;

public class ClimbSubsystem extends SubsystemBase {
    private static ClimbSubsystem instance;

    // Motor, sensor, and servo devices.
    // private final TalonFX pivotMotor = ClimbConstants.pivotMotorConfig.createDevice(TalonFX::new);
    private final MotionMagicTorqueCurrentFOC pivotControl = new MotionMagicTorqueCurrentFOC(0);
    // private final CANcoder pivotEncoder = ClimbConstants.pivotEncoderConfig.createDevice(CANcoder::new);
    private final Servo flapServo = new Servo(ClimbConstants.flapServoPort);
    private final Servo rachetServo = new Servo(ClimbConstants.rachetServoPort);

    // private final StatusSignal<Angle> pivotAngleStatus = pivotMotor.getPosition();
    private final Debouncer pivotDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    private boolean pivotAtPosition = false;

    private final VoltageOut tempCurrentControl = new VoltageOut(0);

    // Define the states of the climber.
    public enum ClimbState {
        STORE,
        DEPLOY,
        CLIMB,
        CONTROL
    }

    private ClimbState currentState = ClimbState.STORE;

    // Target angles for pivot and flap (tunable via ClimbConstants).
    private Angle targetPivotAngle = ClimbConstants.pivotStoreAngle;
    private Angle targetFlapAngle = ClimbConstants.flapStoreAngle;

    private boolean rachetActive = false;

    public static ClimbSubsystem getInstance() {
        if (instance == null) instance = new ClimbSubsystem();
        return instance;
    }
    
    private ClimbSubsystem() {
        // pivotMotor.setControl(pivotControl);
    }

    public void requestStorePivot() {
        targetPivotAngle = ClimbConstants.pivotStoreAngle;
        currentState = ClimbState.STORE;
    }

    public void requestDeployPivot() {
        targetPivotAngle = ClimbConstants.pivotDeployAngle;
        currentState = ClimbState.DEPLOY;
    }

    public void requestClimbPivot() {
        targetPivotAngle = ClimbConstants.pivotClimbAngle;
        currentState = ClimbState.CLIMB;
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

    public void requestRachetActive() {
        rachetActive = true;
    }

    public void requestRachetInActive() {
        rachetActive = false;
    }

    public void setRawCurrent(double rawInput) {
       double output = Math.copySign(rawInput * rawInput, rawInput) * 12;
       tempCurrentControl.withOutput(output);
       SmartDashboard.putNumber("Climb/VoltageOut", output);
    //    pivotMotor.setControl(tempCurrentControl);
    }

    public void modifyPivotAngle(Angle delta) {
        targetPivotAngle = targetPivotAngle.plus(delta);
        currentState = ClimbState.CONTROL;
    }

    @Override
    public void periodic() {
//        pivotControl.withPosition(targetPivotAngle);
        // pivotMotor.setControl(pivotControl);

        setRawCurrent(ControlBoard.getInstance().getLeftVertical());
        flapServo.set(targetFlapAngle.in(Units.Rotations));

        // pivotAngleStatus.refresh(false);

        rachetServo.set((rachetActive ? ClimbConstants.rachetActive : ClimbConstants.rachetInActive).in(Revolutions));

        // pivotAtPosition = pivotDebouncer.calculate(pivotAngleStatus.getValue().isNear(targetPivotAngle, 0.02)); // 0.02 revolutions tolerance

        SmartDashboard.putString("Climb/Current State", currentState.name());
        // SmartDashboard.putNumber("Climb/Pivot Angle", pivotAngleStatus.getValue().in(Units.Degrees));
    }

    public boolean pivotAtPosition() {
        return pivotAtPosition;
    }
}
