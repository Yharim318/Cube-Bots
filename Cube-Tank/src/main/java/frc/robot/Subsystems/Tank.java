// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;

public class Tank extends SubsystemBase {
  private final TalonSRX FrontLeft = new TalonSRX(SwerveConstants.Left.Front);
  private final TalonSRX BackLeft = new TalonSRX(SwerveConstants.Left.Back);
  private final TalonSRX FrontRight = new TalonSRX(SwerveConstants.Right.Front);
  private final TalonSRX BackRight = new TalonSRX(SwerveConstants.Right.Back);
  public Tank() {
    FrontLeft.setInverted(SwerveConstants.Left.FrontReversed);
    BackLeft.setInverted(SwerveConstants.Left.BackReversed);
    FrontRight.setInverted(SwerveConstants.Right.FrontReversed);
    BackRight.setInverted(SwerveConstants.Right.BackReversed);
  }
  public void drive(double LeftAxis, double RightAxis){
    FrontLeft.set(ControlMode.PercentOutput, LeftAxis);
    BackLeft.set(ControlMode.PercentOutput, LeftAxis);
    FrontRight.set(ControlMode.PercentOutput, RightAxis);
    BackRight.set(ControlMode.PercentOutput, RightAxis);
  }
}
