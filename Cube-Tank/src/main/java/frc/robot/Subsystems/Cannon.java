// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CannonConstants;

public class Cannon extends SubsystemBase {
  /** Creates a new Cannon. */
  AnalogInput sensor = new AnalogInput(CannonConstants.sensor);

  Relay compressor1 = new Relay(CannonConstants.compressor1);
  Relay compressor2 = new Relay(CannonConstants.compressor2);
  Relay gardenHose = new Relay(CannonConstants.gardenHose);

  VictorSPX pivot = new VictorSPX(CannonConstants.pivot);

  public Cannon() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(sensor.getValue());
  }
  public void compress(Value value){
    compressor1.set(value);
    compressor2.set(value);
  }
  public void hose(Value value){
    gardenHose.set(value);
  }
}
