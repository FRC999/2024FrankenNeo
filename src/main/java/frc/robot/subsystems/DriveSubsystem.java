// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.RobotDriveChassisConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  private CANSparkMax rightMotor = new CANSparkMax(21, MotorType.kBrushless);
  private CANSparkMax leftMotor = new CANSparkMax(20, MotorType.kBrushless);

  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;

  private DifferentialDrive drive;

  private DifferentialDriveOdometry odometry;

  public DriveSubsystem() {

    rightEncoder = rightMotor.getEncoder(Type.kHallSensor, 42);
    leftEncoder = leftMotor.getEncoder(Type.kHallSensor, 42);

    driveTrainBrakeMode();

    //configureEncoders();

    configureMotors();

    zeroDriveEncoders();

    drive = new DifferentialDrive(leftMotor, rightMotor);
  }



  public void manualDrive(double move, double turn) {
    
    // If joysticks will prove to be too sensitive near the center, turn on the deadband driving
    
    // drive.arcadeDrive(deadbandMove(move), deadbandTurn(turn));
    // System.out.println("D X "+move + " Y " + turn);
    //drive.arcadeDrive(move, turn);
    drive.arcadeDrive(move, turn);
  }

  public void configureMotors() {
    rightMotor.setInverted(false);
    leftMotor.setInverted(true);
  }
  public void driveTrainBrakeMode() {

    rightMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setIdleMode(IdleMode.kBrake);
  }

  public void zeroDriveEncoders() {  // zero encoders on master mmotor controllers of the drivetrain

    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

    /** Get the number of tics moved by the left encoder */
    public double getLeftEncoder() {
      return leftEncoder.getPosition();
    }
  
    /** Get the number of tics moved by the left encoder */
    public double getRightEncoder() {
      return rightEncoder.getPosition();
    }

    public void stopRobot() {
      leftMotor.set(0);
      rightMotor.set(0);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {

      //System.out.println("TV L:" + leftVolts + " R:" + rightVolts);
  
      leftMotor.set(leftVolts);
      rightMotor.set(rightVolts);
      drive.feed();
    }

    public Pose2d getPose() {
      return odometry.getPoseMeters();
    }

    public double TranslateVelocityIntoMetersPerSecond(double velocityRawUnits) {
      // Raw units - ticks per 100ms
      return Units.inchesToMeters((velocityRawUnits * 10) / RobotDriveChassisConstants.tickPerInch) ;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() { // needs to be meters per second
      return new DifferentialDriveWheelSpeeds(
          TranslateVelocityIntoMetersPerSecond(getLeftEncoder()),
          TranslateVelocityIntoMetersPerSecond(getRightEncoder())
      );
    }

    

    
  



  // // Configure encoders on primary motors
  // public void configureEncoders(){
  //   leftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
  //   rightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
