// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class HardwarePIDMove extends CommandBase {
  /** Creates a new HardwarePIDMove. */
  public HardwarePIDMove() {
    addRequirements(RobotContainer.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("***HardwarePID Starting***");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.driveSubsystem.driveToDistanceInMeters(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Math.abs(RobotContainer.driveSubsystem.getLeftEncoder() - RobotContainer.driveSubsystem.metersToTicks(1)) < Constants.DriveConstants.PIDtolerance && 
      Math.abs(RobotContainer.driveSubsystem.getRightEncoder() - RobotContainer.driveSubsystem.metersToTicks(1)) < Constants.DriveConstants.PIDtolerance)); 
      
  }
}
