// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveWheelConstants.kMotors;

public class SwerveWheel extends SubsystemBase {
  /** Creates a new SwerveWheel. */
  public TalonFX driveMotor; // Motor controller object for the drive motor
  public TalonFX angleMotor; // Motor controller object fo the turning motor
  public CANcoder encoder;
  public PIDController swervePID; // PID controller object for error correction
  public CANcoderConfiguration config = new CANcoderConfiguration();
  
  public SwerveWheel(kMotors module) {
    swervePID = new PIDController(
      Constants.SwerveWheelConstants.PIDConstants.kP,
      Constants.SwerveWheelConstants.PIDConstants.kI,
      Constants.SwerveWheelConstants.PIDConstants.kD
      );

    switch (module){
      case FL:
        driveMotor = new TalonFX(Constants.SwerveWheelConstants.FrontWheels.LeftDrive);
        angleMotor = new TalonFX(Constants.SwerveWheelConstants.FrontWheels.LeftAngle);
        encoder = new CANcoder(Constants.SwerveWheelConstants.FrontWheels.LeftCancoder);
        config.MagnetSensor.MagnetOffset = Constants.SwerveWheelConstants.FrontWheels.LeftCancoderOffset;
        encoder.getConfigurator().apply(config);
        break;
      case FR:
        driveMotor = new TalonFX(Constants.SwerveWheelConstants.FrontWheels.RightDrive);
        angleMotor = new TalonFX(Constants.SwerveWheelConstants.FrontWheels.RightAngle);
        encoder = new CANcoder(Constants.SwerveWheelConstants.FrontWheels.RightCancoder);
        config.MagnetSensor.MagnetOffset = Constants.SwerveWheelConstants.FrontWheels.RightCancoderOffset;
        encoder.getConfigurator().apply(config);
        break;
      case BL:
        driveMotor = new TalonFX(Constants.SwerveWheelConstants.BackWheels.LeftDrive);
        angleMotor = new TalonFX(Constants.SwerveWheelConstants.BackWheels.LeftAngle);
        encoder = new CANcoder(Constants.SwerveWheelConstants.BackWheels.LeftCancoder);
        config.MagnetSensor.MagnetOffset = Constants.SwerveWheelConstants.BackWheels.LeftCancoderOffset;
        encoder.getConfigurator().apply(config);
        break;
      case BR:
        driveMotor = new TalonFX(Constants.SwerveWheelConstants.BackWheels.RightDrive);
        angleMotor = new TalonFX(Constants.SwerveWheelConstants.BackWheels.RightAngle);
        encoder = new CANcoder(Constants.SwerveWheelConstants.BackWheels.RightCancoder);
        config.MagnetSensor.MagnetOffset = Constants.SwerveWheelConstants.BackWheels.RightCancoderOffset;
        encoder.getConfigurator().apply(config);
        break;
    }
  }
  public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition().getValue());
  }
  public void setDriveSpeed(double speed) {
    driveMotor.set(speed);
}

  public void setSwerveSpeed(double speed) {  
    angleMotor.set(speed);
}
  public void updatePID(SwerveModuleState wheelState, XboxController xbox) {
        // This takes the given SwerveModuleState and optimizes it
        // For example, if the wheel were facing at 90 degrees, and had to be moving forward at -75 degrees, it would rotate the wheel to 105 degrees and reverse its direction
        // It limits the rotational distance the wheel needs to cover
        SwerveModuleState optimizedWheelState;
        if (xbox.getAButton()) {
            optimizedWheelState = wheelState;
        }
        else {
            optimizedWheelState = SwerveModuleState.optimize(wheelState, getRotation());
        }
        // This sets the setpoint of the PID loop to the angle determined above
        swervePID.setSetpoint(optimizedWheelState.angle.getDegrees());

        angleMotor.set( // Sets the output of the rotation motor
            MathUtil.applyDeadband(MathUtil.clamp( // Clamps PID output to between -1 and 1 to keep it in bounds
                swervePID.calculate(getRotation().getDegrees()), -1, 1), Constants.deadband)); // The calculate command calculates the next iteration of the PID loop given the current angle of the wheel

        driveMotor.set(
          MathUtil.applyDeadband( // This sets the speed of the wheel to the speed assigned by the optimized SwerveModuleState
          optimizedWheelState.speedMetersPerSecond
            * // and multiplies it by a global maxSpeed multiplier
          Constants.SwerveWheelConstants.ChassisConstants.maxDriveSpeed,
          Constants.deadband)
        ); 
  }
}
