// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Subsystems.Swerve;

public class RobotContainer { 

  XboxController driver = new XboxController(0);
  JoystickButton xButton = new JoystickButton(driver, XboxController.Button.kX.value);
  JoystickButton gyroButton= new JoystickButton(driver, XboxController.Button.kY.value);

  public Swerve swerve = new Swerve(driver);
  public RobotContainer() {
    swerve.setDefaultCommand(new TeleopSwerve(swerve));
    configureBindings();
  }
  private void configureBindings() {
    xButton.whileTrue(new InstantCommand(() -> swerve.update(
      new SwerveModuleState[]{
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45))
      }
      )
    ));
    gyroButton.onTrue(new InstantCommand(() -> swerve.resetGyro()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
