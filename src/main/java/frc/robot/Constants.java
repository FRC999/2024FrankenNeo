// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class DriveConstants {
    public static final double ksVolts = 0.1231;
    public static final double kvVoltSecondsPerMeter = 1.0882;   //Copied over from old code
    public static final double kaVoltSecondsSquaredPerMeter = 0.092837;

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        Units.inchesToMeters(RobotDriveChassisConstants.distanceBetweenWheels));
    public static final double trajectoryRioPidP_Value = 5.19693;
    public static final double trajectoryRioPidD_Value = 0;        //Copied over from old code
    public static final double trajectoryRioPidI_Value = 0.04;


    public static final double maxVelocityDefault = 3;
    public static final double maxAccelerationDefault = 2; 
   

    public static final class RobotDriveChassisConstants { // configure the physical properties unique to the robot
     
    public static final double wheelDiameter = 3.75; // inches
    public static final double distanceBetweenWheels = 28.5; // inches


  }


}
}
