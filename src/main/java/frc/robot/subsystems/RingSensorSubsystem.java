// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RingSensorSubsystem extends SubsystemBase {

  private final DigitalInput input;

  /** Creates a new RingSensorSubsystem. */
  public RingSensorSubsystem() {
    this.input = new DigitalInput(9);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("RING INPUT", this.input.get());
    // This method will be called once per scheduler run
  }

  public boolean ringDetected() {
    return !input.get();
  }
}
