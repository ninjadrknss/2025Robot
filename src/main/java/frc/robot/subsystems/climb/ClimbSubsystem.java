package frc.robot.subsystems.climb;


import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private static ClimbSubsystem instance;

    /* Motor, Controls and Sensor */
//    private final TalonFX pivotMotor = ClimbConstants.pivotMotorConfig.createMotor();
//    private final PositionTorqueCurrentFOC pivotControl = new PositionTorqueCurrentFOC(0);
//
//    private final CANcoder pivotEncoder = new CANcoder(ClimbConstants.pivotEncoderID);
//
//    private final Servo servo = new Servo(ClimbConstants.servoPort);

    /* Other variables */
    private boolean requestStorePivot = false;
    private boolean requestDeployPivot = false;

    private boolean requestStoreFlap = false;
    private boolean requestDeployFlap = false;

    public static ClimbSubsystem getInstance() {
        if (instance == null) instance = new ClimbSubsystem();
        return instance;
    }

    private ClimbSubsystem() {
//        pivotEncoder.getConfigurator().apply(ClimbConstants.pivotEncoderConfig);
    }

    private void setPivotAngle(double angle) {
        // TODO: set control request angle to angle
        // TODO: set control of motor to control request
    }

    /**
     * Set the angle of the flap in degrees.
     * @param angle The angle of the flap in degrees.
     */
    private void setFlapAngle(double angle) {
        // TODO: set servo angle to angle
    }

    @Override
    public void periodic() {
        if (requestStorePivot) {
            // TODO: store pivot
            requestStorePivot = false;
        }
        if (requestDeployPivot) {
            // TODO: deploy pivot
            requestDeployPivot = false;
        }
        if (requestStoreFlap) {
            // TODO: store flap
            requestStoreFlap = false;
        }
        if (requestDeployFlap) {
            // TODO: deploy flap
            requestDeployFlap = false;
        }
    }

    public void requestStorePivot() {
        requestStorePivot = true;
        requestDeployPivot = false;
    }

    public void requestDeployPivot() {
        requestStorePivot = false;
        requestDeployPivot = true;
    }

    public void requestStoreFlap() {
        requestStoreFlap = true;
        requestDeployFlap = false;
    }

    public void requestDeployFlap() {
        requestStoreFlap = false;
        requestDeployFlap = true;
    }
}

