// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Climber.*;

public class ClimberSubsystem extends SubsystemBase {

  private final CANSparkMax climber;
  private final RelativeEncoder encoder;

  private double speed = 0.0;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    this.climber = new CANSparkMax(CLIMBER_MOTOR, MotorType.kBrushless);
    this.encoder = climber.getEncoder();
    this.encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("POSITION", this.encoder.getPosition());
    // if((this.encoder.getPosition() >= 0.09 && speed > 0) || (this.encoder.getPosition() <= -65 && speed < 0)) { 
    //   climber.set(0);
    //   return;
    // }
    climber.set(this.speed);
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }
}
