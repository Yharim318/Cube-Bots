// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.TeleopTank;
import frc.robot.Subsystems.Cannon;
import frc.robot.Subsystems.Tank;

public class RobotContainer {

  private Joystick driver = new Joystick(SwerveConstants.DriverPort);
  private Cannon cannon = new Cannon();
  private int leftAxis = XboxController.Axis.kLeftY.value;
  private int rightAxis = XboxController.Axis.kRightY.value;

  private Tank tank = new Tank();


  public RobotContainer() {
    tank.setDefaultCommand(
      new TeleopTank(
        tank,
        () -> driver.getRawAxis(leftAxis),
        () -> driver.getRawAxis(rightAxis)
      )
    );
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
