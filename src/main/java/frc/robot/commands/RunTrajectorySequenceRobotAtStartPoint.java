// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunTrajectorySequenceRobotAtStartPoint extends SequentialCommandGroup {
  /** Creates a new RunTrajectorySequenceRobotAtStartPoint. */

  PathPlannerTrajectory trajectoryPath;
  boolean isReversed = false;

  public RunTrajectorySequenceRobotAtStartPoint(String trajectory, double maxVelocity, double maxAcceleration, boolean reversed) {
    trajectoryPath = PathPlanner.loadPath(trajectory, new PathConstraints(maxVelocity, maxAcceleration), reversed);

    addCommands(
      new AutonomousTrajectoryRioCommand(trajectoryPath) // Run a trajectory
    );
  }

  public RunTrajectorySequenceRobotAtStartPoint(String trajectory) {

    this(trajectory, false);
    System.out.println("*** Run trajectory non-reversed"+ trajectory);
  }

  public RunTrajectorySequenceRobotAtStartPoint(String trajectory, boolean reversed) {

    this(trajectory, 2.0, 1.5, reversed);
    //this(trajectory, DriveConstants.maxVelocityDefault, DriveConstants.maxAccelerationDefault, reversed);
    System.out.println("*** Run trajectory "+ trajectory+" reversed:"+reversed);
  }

}
