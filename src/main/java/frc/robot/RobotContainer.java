/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_driveSubsystem.resetEncoders();
    m_driveSubsystem.resetHeading();
    m_driveSubsystem.resetOdometry();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    m_driveSubsystem.resetEncoders();
    m_driveSubsystem.resetHeading();
    m_driveSubsystem.resetOdometry();

    TrajectoryConfig config = new TrajectoryConfig(3.97350993, 2);

    config.setKinematics(m_driveSubsystem.getKinematics());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(
        new Pose2d(), 
        new Pose2d(3, -2, Rotation2d.fromDegrees(0))
        ), 
      config
    );

    RamseteController disabledRamsete = new RamseteController() {
      @Override
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
              double angularVelocityRefRadiansPerSecond) {
          return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
      }
  };

  
  var m_leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_reference");
  var m_leftMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_measurement");
  var m_rightReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_reference");
  var m_rightMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_measurement");

    RamseteCommand command = new RamseteCommand(
      trajectory,
      m_driveSubsystem::getPose,
      disabledRamsete,//new RamseteController(2.0, 0.7),
      m_driveSubsystem.getFeedForward(),
      m_driveSubsystem.getKinematics(),
      m_driveSubsystem::getWheelSpeeds,
      m_driveSubsystem.getLeftPIDController(),
      m_driveSubsystem.getRightPIDController(),
      (leftVolts, rightVolts) -> {
        m_driveSubsystem.set(leftVolts, rightVolts);

        m_leftMeasurement.setNumber(m_driveSubsystem.getFeedForward().calculate(m_driveSubsystem.getWheelSpeeds().leftMetersPerSecond));
        m_leftReference.setNumber(leftVolts);

        m_rightMeasurement.setNumber(m_driveSubsystem.getFeedForward().calculate(m_driveSubsystem.getWheelSpeeds().rightMetersPerSecond));
        m_rightReference.setNumber(-rightVolts);
    },//m_driveSubsystem::set,
      m_driveSubsystem
    );

    return command;
  }
}
