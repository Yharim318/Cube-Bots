// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.TeleopTank;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.robot.Subsystems.Cannon;
import frc.robot.Subsystems.Tank;

public class RobotContainer {

  private Joystick driver = new Joystick(SwerveConstants.DriverPort);
  private Cannon cannon = new Cannon();
  private JoystickButton compressButton = new JoystickButton(driver, XboxController.Button.kA.value);
  private JoystickButton releaseButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
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

  private void configureBindings() {
    compressButton.whileTrue(new InstantCommand(() -> cannon.compress(Value.kOn)));
    compressButton.whileFalse(new InstantCommand(() -> cannon.compress(Value.kOff)));
    releaseButton.whileTrue(new InstantCommand(() -> cannon.hose(Value.kOn)));
    releaseButton.whileFalse(new InstantCommand(() -> cannon.hose(Value.kOff)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
