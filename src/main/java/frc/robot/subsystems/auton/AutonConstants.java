package frc.robot.subsystems.auton;

public record AutonConstants() {
  public static final double robotMass = 0.0; // kg TODO: solidworks is weird -alek
  public static final double momentOfInertia =
      0.0; // N * m TODO: https://choreo.autos/usage/estimating-moi/

  public static final double bumperFront = 42.0624; // cm
  public static final double bumperSide = 42.3926; // cm
  public static final double bumperBack = bumperFront; // cm

  public static final double wheelRadius = 5.08; // cm TODO: converted from "approx 2in"
  public static final double wheelCOF = 0.0; // TODO: copy from maplesim
  public static final double wheelRev = 6.75;

  public static final double motorMaxSpeed = 4640; // rpm
  public static final double motorMaxTorque = 1.135; // N * m

  public static final double frontModX = 0.0; // cm TODO: we have these values already
  public static final double frontLeftY = 0.0; // cm TODO: we have these values already
  public static final double backModX = 0.0; // cm TODO: we have these values already
  public static final double backLeftY = 0.0; // cm TODO: we have these values already
}
