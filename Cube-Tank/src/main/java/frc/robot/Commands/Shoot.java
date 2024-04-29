// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.Cannon;

public class Shoot extends Command {
  Cannon cannon;
  double finalTime = 0.5;
  double currentTime = 0;
  /** Creates a new Shoot. */
  public Shoot(Cannon Cannon) {
    // Use addRequirements() here to declare subsystem dependencies.
    cannon = Cannon;
    addRequirements(cannon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cannon.hose(Value.kOn);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime += Robot.kDefaultPeriod;
    System.out.println(currentTime);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cannon.hose(Value.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentTime >= finalTime){
      cannon.hose(Value.kOff);
      return true;
    }
    else 
      return false;
  }
}
