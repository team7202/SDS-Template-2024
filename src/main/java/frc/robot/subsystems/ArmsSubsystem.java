// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Arms.ARMS_LEFT_MOTOR;
import static frc.robot.Constants.Arms.ARMS_RIGHT_MOTOR;
import static frc.robot.Constants.Arms.ARMS_ZERO_HIGH_SPEED;
import static frc.robot.Constants.Arms.ARMS_ZERO_LOW_SPEED;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmsSubsystem extends SubsystemBase {

  public final PIDController pid;
  
  private final ShuffleboardTab tab;
  private final CANSparkMax leftArm;
  private final CANSparkMax rightArm;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private double speed = 0.0;

  private double target = 0.0;
  private double lowSpeed = ARMS_ZERO_LOW_SPEED;
  private double highSpeed = ARMS_ZERO_HIGH_SPEED;

  /** Creates a new ArmsSubsysten. */
  public ArmsSubsystem() {
    this.pid = new PIDController(0.05, 0, 0);
    this.pid.setTolerance(0.5);
    this.leftArm = new CANSparkMax(ARMS_LEFT_MOTOR, MotorType.kBrushless);
    this.rightArm = new CANSparkMax(ARMS_RIGHT_MOTOR, MotorType.kBrushless);
    this.leftEncoder = this.leftArm.getEncoder();
    this.leftEncoder.setPosition(0);
    this.rightEncoder = this.rightArm.getEncoder();
    this.rightEncoder.setPosition(0);

    this.tab = Shuffleboard.getTab("Arms");
    this.tab.addNumber("Left Arm Position", () -> this.leftEncoder.getPosition());
    this.tab.addNumber("Right Arm Position", () -> this.rightEncoder.getPosition());
  }

  public void setTarget(double target, double lowSpeed, double highSpeed) {
    this.target = target;
    this.lowSpeed = lowSpeed;
    this.highSpeed = highSpeed;
  }

  public double[] getPositions() {
    double[] positions = { this.leftEncoder.getPosition(), this.rightEncoder.getPosition() };
    return positions;
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public void stop() {
    this.speed = 0;
  }

  @Override
  public void periodic() {
     double speed = MathUtil.clamp(this.pid.calculate(this.leftEncoder.getPosition(), target), lowSpeed, highSpeed);
    SmartDashboard.putNumber("arms target", target);
    SmartDashboard.putNumber("arms speed", speed);
    this.leftArm.set(speed);
    this.rightArm.set(speed);
  }
}
