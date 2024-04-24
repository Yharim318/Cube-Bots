// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveConstants;
import frc.robot.Subsystems.Tank;

public class TeleopTank extends Command {
  private Tank tank;
  private DoubleSupplier leftAxisSup;
  private DoubleSupplier rightAxisSup;
  public TeleopTank(Tank Tank, DoubleSupplier LeftAxisSup, DoubleSupplier RightAxisSup) {
    tank = Tank;
    addRequirements(tank);
    leftAxisSup = LeftAxisSup;
    rightAxisSup = RightAxisSup;
  }
  @Override
  public void execute() {
    double leftAxis = MathUtil.applyDeadband(leftAxisSup.getAsDouble(), SwerveConstants.StickDeadband);
    double rightAxis = MathUtil.applyDeadband(rightAxisSup.getAsDouble(), SwerveConstants.StickDeadband);
    tank.drive(leftAxis, rightAxis);
  }
}
