// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import static frc.robot.Constants.ROBOT_DATA.*;
import static frc.robot.Constants.INT_PORT.*;

public class DriveBase extends SubsystemBase {
  public WPI_TalonSRX rightMaster = new WPI_TalonSRX(rightmaster);
  public WPI_TalonSRX rightFollow = new WPI_TalonSRX(rightmaster);
  public WPI_TalonSRX leftMaster = new WPI_TalonSRX(rightmaster);
  public WPI_TalonSRX leftFollow = new WPI_TalonSRX(rightmaster);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(track_width);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.268, 1.89, 0.243);

  AHRS gyro = new AHRS(SPI.Port.kMXP);

  Pose2d pose = new Pose2d();

  PIDController leftController = new PIDController(9.95, 0, 0);
  PIDController righController = new PIDController(9.95, 0, 0);

  /** Creates a new DriveBase. */
  public DriveBase() {
    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftFollow.setNeutralMode(NeutralMode.Brake);
    rightFollow.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
  
    leftFollow.follow(leftMaster);
    rightFollow.follow(rightMaster);
  
    leftMaster.setInverted(false);
    leftFollow.setInverted(false);

    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  public void drive (double right_velocity, double left_velocity){
    rightMaster.set(right_velocity);
    leftMaster.set(left_velocity);
  }

  public Rotation2d getHeading () {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public SimpleMotorFeedforward getFeedforward () {
    return feedforward;
  }

  public PIDController getRightcontroller() {
    return righController;
  }

  public PIDController getLeftcontroller() {
    return leftController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  public Pose2d getPose() {
    return pose;
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
    leftMaster.getSelectedSensorVelocity(), 
    rightMaster.getSelectedSensorVelocity()
    );
  }

  public void setOutput(double leftVolts, double rightVolts) {
    leftMaster.set(leftVolts / 12);
    rightMaster.set(rightVolts / 12);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(
    getHeading(),
    leftMaster.getSelectedSensorVelocity() / left_motor_gear_ratio * 2 * Math.PI * wheel_radius / 60, 
    rightMaster.getSelectedSensorPosition() / right_motor_gear_ratio * 2 * Math.PI * wheel_radius / 60
    );
  }
}
