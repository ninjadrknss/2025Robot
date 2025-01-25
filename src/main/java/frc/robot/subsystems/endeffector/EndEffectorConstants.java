package frc.robot.subsystems.endeffector;

import au.grapplerobotics.interfaces.LaserCanInterface;

public class EndEffectorConstants {
    public final static int LaserCanID = 0;
    public final static LaserCanInterface.RegionOfInterest roi = new LaserCanInterface.RegionOfInterest(
            0, 0, 100, 100
    );
}
