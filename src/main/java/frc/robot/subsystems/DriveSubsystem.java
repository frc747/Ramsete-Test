/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  
  
  TalonFX leftDrivePrimary = new TalonFX(15);
  TalonFX leftDriveBack = new TalonFX(14);
  TalonFX rightDrivePrimary = new TalonFX(0);
  TalonFX rightDriveBack = new TalonFX(1);

  PigeonIMU pigeon = new PigeonIMU(0);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.distBetweenWheelsInches));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);

  PIDController leftPIDController = new PIDController(Constants.kP, 0.0, 0.0);

  PIDController rightPIDController = new PIDController(Constants.kP, 0.0, 0.0);

  Pose2d pose;
  
  public DriveSubsystem() {
    leftDrivePrimary.setInverted(true);
    leftDriveBack.setInverted(true);
    rightDrivePrimary.setInverted(false);
    rightDriveBack.setInverted(false);

    leftDriveBack.follow(leftDrivePrimary);
    rightDriveBack.follow(rightDrivePrimary);
  }

  public double getYaw() {
    double ypr[] = {0,0,0};
    pigeon.getYawPitchRoll(ypr);
    return Math.IEEEremainder(ypr[0], 360.0d);
  }

  public Rotation2d getHeading() {
    double ypr[] = {0,0,0};
    pigeon.getYawPitchRoll(ypr);
    return Rotation2d.fromDegrees(Math.IEEEremainder(ypr[0], 360.0d));
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedForward;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public void set(double leftVoltage, double rightVoltage) {
    System.out.println("L: " + leftVoltage);
    System.out.println("R: " + rightVoltage);
    leftDrivePrimary.set(ControlMode.PercentOutput, leftVoltage/12);
    rightDrivePrimary.set(ControlMode.PercentOutput, rightVoltage/12);
  }

  public Pose2d getPose() {
    return pose;
  }

  public void resetEncoders() {
    leftDrivePrimary.setSelectedSensorPosition(0);
    rightDrivePrimary.setSelectedSensorPosition(0);
  }

  public void resetHeading() {
    pigeon.setYaw(0);
  }

  public void resetOdometry() {
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftDrivePrimary.getSelectedSensorVelocity() * Constants.gearRatio * 10.0 / Constants.encoderTicksPerRev * Units.inchesToMeters(Constants.wheelCircumferenceInches),
      rightDrivePrimary.getSelectedSensorVelocity() * Constants.gearRatio * 10.0 / Constants.encoderTicksPerRev * Units.inchesToMeters(Constants.wheelCircumferenceInches)
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(getHeading(), leftDrivePrimary.getSelectedSensorPosition() / Constants.encoderTicksPerRev * Constants.gearRatio * Units.inchesToMeters(Constants.wheelCircumferenceInches), rightDrivePrimary.getSelectedSensorPosition() / Constants.encoderTicksPerRev * Constants.gearRatio * Units.inchesToMeters(Constants.wheelCircumferenceInches));
    
    SmartDashboard.putNumber("left encoder pos", leftDrivePrimary.getSelectedSensorPosition());
    SmartDashboard.putNumber("right encoder pos", rightDrivePrimary.getSelectedSensorPosition());

    SmartDashboard.putNumber("heading", getYaw());

    SmartDashboard.putNumber("left encoder vel", leftDrivePrimary.getSelectedSensorVelocity());
    SmartDashboard.putNumber("right encoder vel", rightDrivePrimary.getSelectedSensorVelocity());

    SmartDashboard.putNumber("X pose", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Y pose", odometry.getPoseMeters().getTranslation().getY());
  }
}
