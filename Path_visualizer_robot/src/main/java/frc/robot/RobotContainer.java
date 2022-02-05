// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveBase;
import frc.robot.Robot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private DriveBase drivebase = new DriveBase();
  
  // The robot's subsystems and commands are defined here...
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    TrajectoryConfig config = new TrajectoryConfig(2, 2);
    config.setKinematics(drivebase.getKinematics());
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(),new Pose2d(1.0,0, new Rotation2d())),
      config);
      /*RamseteCommand command = new RamseteCommand(
        Robot.trajectory, 
        drivebase::getPose, 
        new RamseteController(2.0, 0.7), 
        drivebase.getFeedforward(), 
        drivebase.getKinematics(), 
        drivebase::getSpeeds, 
        drivebase.getLeftcontroller(), 
        drivebase.getRightcontroller(), 
        drivebase::setOutput, 
        drivebase
        ); */
      RamseteCommand command = new RamseteCommand(
      trajectory, 
      drivebase::getPose, 
      new RamseteController(2.0, 0.7), 
      drivebase.getFeedforward(), 
      drivebase.getKinematics(), 
      drivebase::getSpeeds, 
      drivebase.getLeftcontroller(), 
      drivebase.getRightcontroller(), 
      drivebase::setOutput, 
      drivebase
      );
    return command;
  }
}
