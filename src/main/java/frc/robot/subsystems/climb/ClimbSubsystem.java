package frc.robot.subsystems.climb;


import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.CTREUtil;

import static edu.wpi.first.units.Units.Degrees;

public class ClimbSubsystem extends SubsystemBase {
    private static ClimbSubsystem instance;

    /* Motor, Controls and Sensor */
//    private final TalonFX pivotMotor = ClimbConstants.pivotMotorConfig.createDevice(TalonFX::new);
//    private final PositionTorqueCurrentFOC pivotControl = new PositionTorqueCurrentFOC(0);
//
//    private final CANcoder pivotEncoder = ClimbConstants.pivotEncoderConfig.createDevice(CANcoder::new);
//
//    private final Servo servo = new Servo(ClimbConstants.servoPort);

    /* Other variables */
    private Angle requestedPivotAngle = Degrees.of(0);
    private boolean pendingPivotChange = false;

    private boolean requestStoreFlap = false;
    private boolean requestDeployFlap = false;

    public static ClimbSubsystem getInstance() {
        if (instance == null) instance = new ClimbSubsystem();
        return instance;
    }

    private ClimbSubsystem() {
    }

    private void setPivotAngle(Angle angle) {
        // TODO: set control request angle to angle
        // TODO: set control of motor to control request
//        pivotControl.withPosition(angle);
        requestedPivotAngle = angle;
//        pivotMotor.setControl(pivotControl);
    }

    /**
     * Set the angle of the flap in degrees.
     * @param angle The angle of the flap in degrees.
     */
    private void setFlapAngle(Angle angle) {
        // TODO: set servo angle to angle
//        servo.set(angle.in(Degrees));
    }

    @Override
    public void periodic() {
        if (pendingPivotChange) {
            pendingPivotChange = false;
            setPivotAngle(requestedPivotAngle);
        }
        if (requestStoreFlap) {
            // TODO: store flap
            setFlapAngle(ClimbConstants.flapStoreAngle);
            requestStoreFlap = false;
        }
        if (requestDeployFlap) {
            // TODO: deploy flap
            setFlapAngle(ClimbConstants.flapDeployAngle);
            requestDeployFlap = false;
        }
    }

    public void requestStorePivot() {
        pendingPivotChange = true;
        requestedPivotAngle = ClimbConstants.pivotStoreAngle;
    }

    public void requestDeployPivot() {
        pendingPivotChange = true;
        requestedPivotAngle = ClimbConstants.pivotDeployAngle;
    }

    public void requestStoreFlap() {
        requestStoreFlap = true;
        requestDeployFlap = false;
    }

    public void requestDeployFlap() {
        requestStoreFlap = false;
        requestDeployFlap = true;
    }

    public void increasePivotAngle() {
        pendingPivotChange = true;
        requestedPivotAngle.plus(ClimbConstants.changeRate);
        if (requestedPivotAngle.lt(ClimbConstants.pivotDeployAngle)) requestedPivotAngle = ClimbConstants.pivotDeployAngle;
    }

    public void decreasePivotAngle() {
        pendingPivotChange = true;
        requestedPivotAngle.minus(ClimbConstants.changeRate);
        if (requestedPivotAngle.gt(ClimbConstants.pivotStoreAngle)) requestedPivotAngle = ClimbConstants.pivotStoreAngle;
    }
}

