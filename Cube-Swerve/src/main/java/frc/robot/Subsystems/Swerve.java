// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveWheelConstants.kMotors;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */
  SwerveWheel wheelFL;
  SwerveWheel wheelFR;
  SwerveWheel wheelBR;
  SwerveWheel wheelBL;

  Pigeon2 gyro = new Pigeon2(Constants.gyro);

  SwerveDriveKinematics swerveDrive = Constants.SwerveWheelConstants.ChassisConstants.swerveKinematics;

  double limit = 100.0;
  SlewRateLimiter xLimit = new SlewRateLimiter(limit);
  SlewRateLimiter yLimit = new SlewRateLimiter(limit);

  ChassisSpeeds goalSpeed;
  SwerveModuleState[] goalStates;
  Translation2d centerRotation = new Translation2d(0, 0);

  public XboxController driver;
  public Swerve(XboxController driver) {
    this.driver = driver;

    wheelFL = new SwerveWheel(kMotors.FL);
    wheelFR = new SwerveWheel(kMotors.FR);
    wheelBR = new SwerveWheel(kMotors.BR);
    wheelBL = new SwerveWheel(kMotors.BL);

    gyro.getConfigurator().apply(new Pigeon2Configuration());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    goalSpeed = ChassisSpeeds.fromFieldRelativeSpeeds( // This static method generates a new ChassisSpeeds object based on given velocities
      MathUtil.applyDeadband(yLimit.calculate(driver.getRawAxis(XboxController.Axis.kLeftY.value)), Constants.deadband), // Xbox controller
      MathUtil.applyDeadband(xLimit.calculate(driver.getRawAxis(XboxController.Axis.kLeftX.value)), Constants.deadband), // Xbox controller
      // While the input speeds are technically supposed to be m/s, it is easier to assume max joystick is 1 m/s

      // Because of the above assumption, the rotation joystick needs to be scaled to balance movement with rotation
      // For example, without the scalar, the robot would rotate at 1 rad/s at maximum rotate (and thus take the same amount of time to rotate once as to travel six meters)
      // The scalar exists to customize this to fit user need
      MathUtil.applyDeadband(driver.getRawAxis(XboxController.Axis.kRightX.value) * 0.05, Constants.deadband/5), // Xbox controller

      // This is needed because of the field relative nature of this object; if a gyro is used, this should be the gyro
      // If used with a gyro, this allows for the joystick to operate the robot in the same directions regardless of robot orientation
      // i.e. Up on the left joystick always moves the robot away from the user regardless of its rotation
      Rotation2d.fromDegrees(gyro.getYaw().getValue())
    );

    goalStates = swerveDrive.toSwerveModuleStates(goalSpeed, centerRotation);

    SwerveDriveKinematics.desaturateWheelSpeeds(goalStates, 1);
  }
  public SwerveModuleState[] GetGoalStates(){
    return goalStates;
  }
  public void resetGyro(){
    gyro.reset();
  }

  public void update(SwerveModuleState[] swerveModuleStates){
    wheelFL.updatePID(swerveModuleStates[0], driver);
    wheelFR.updatePID(swerveModuleStates[1], driver);
    wheelBR.updatePID(swerveModuleStates[3], driver);
    wheelBL.updatePID(swerveModuleStates[2], driver);
    System.out.println();
  }
  public void update(){
    wheelFL.updatePID(goalStates[0], driver);
    wheelFR.updatePID(goalStates[1], driver);
    wheelBR.updatePID(goalStates[3], driver);
    wheelBL.updatePID(goalStates[2], driver);
    System.out.println();
  }
}
