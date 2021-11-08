// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class DriveTrain extends SubsystemBase {
  
  TalonFX left1 = new TalonFX(0);
  TalonFX left2 = new TalonFX(2);
  TalonFX right1 = new TalonFX(1);
  TalonFX right2 = new TalonFX(3);

  /** Creates a new DriveTrain. */
  public DriveTrain() {}

  public void drive(double joystickY, double joystickZ){
    left1.set(ControlMode.PercentOutput, joystickY+joystickZ);
    left2.set(ControlMode.PercentOutput, joystickY+joystickZ);
    right1.set(ControlMode.PercentOutput, -(joystickY-joystickZ));
    right2.set(ControlMode.PercentOutput, -(joystickY-joystickZ));
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}