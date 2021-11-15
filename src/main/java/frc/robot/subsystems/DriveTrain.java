// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import java.lang.Math;

public class DriveTrain extends SubsystemBase {
  double Xposition=0, Yposition=0;
  double distance=0, gyroAngle=0;

  double previousDistance = 0;
  double frameDistance = 0;
  
  double[] talonSpeeds = {0,0};
  
  TalonFX left1 = new TalonFX(0);
  TalonFX left2 = new TalonFX(2);
  TalonFX right1 = new TalonFX(1);
  TalonFX right2 = new TalonFX(3);

  AHRS gyro = new AHRS();

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    Shuffleboard.getTab("Position").addNumber("Xposition", () -> Xposition);
    Shuffleboard.getTab("Position").addNumber("Yposition", () -> Yposition);
    Shuffleboard.getTab("Position").addNumber("Gyro Angle", () -> gyroAngle);
    Shuffleboard.getTab("Position").addNumber("Frame Distance", () -> frameDistance);
    Shuffleboard.getTab("Position").addNumber("Previous Distance", () -> previousDistance);
  }

  public void drive(double joystickY, double joystickZ){
    left1.set(ControlMode.PercentOutput, -(joystickY-joystickZ));
    left2.set(ControlMode.PercentOutput, -(joystickY-joystickZ));
      talonSpeeds[0] = -(joystickY-joystickZ);

    right1.set(ControlMode.PercentOutput, (joystickY+joystickZ));
    right2.set(ControlMode.PercentOutput, (joystickY+joystickZ));
      talonSpeeds[1] = (joystickY+joystickZ);
  } 

  @Override
  public void periodic() {

    gyroAngle = gyro.getAngle();

    double currentDistance =( (
        left1.getSelectedSensorPosition() + right1.getSelectedSensorPosition() 
        + 
        left2.getSelectedSensorPosition() + right2.getSelectedSensorPosition()
      ) / 2)/Constants.EncoderValueScale ;

    double currentDelta = currentDistance - previousDistance;
    frameDistance = currentDelta;

    // Calculating positions
    Xposition += currentDelta*(
      Math.toDegrees(
        Math.sin(
          Math.toRadians(gyroAngle)
        )
      )
    );
  
    Yposition += currentDelta*(
      Math.toDegrees(
        Math.cos(
          Math.toRadians(gyroAngle)
        )
      )
    );  

    previousDistance = currentDistance;
  }

  // TODO Create functions for making the robot drive to certain coordinates
}
