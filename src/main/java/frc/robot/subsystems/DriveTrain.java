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

  public double Xposition=0, Yposition=0;
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

  public void drive(double[] speeds){
    left1.set(ControlMode.PercentOutput, speeds[0]);
    left2.set(ControlMode.PercentOutput, speeds[0]);
    right1.set(ControlMode.PercentOutput, speeds[1]);
    right2.set(ControlMode.PercentOutput, speeds[1]);
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
  }
  /**
   * This function returns the child vector as relative
   * to the parent vector
   * @param Ax X position of parent vector
   * @param Ay Y position of parent vector
   * @param Bx X position child vector
   * @param By Y position child vector
   * @param theta Gyro value of the robot
   */
  public double[] rPoints(double Ax, double Ay,double Bx,double By,double theta){
    double[] out = {0,0};
    
    double s = Math.sqrt(
      ((Bx-Ax)*(Bx-Ax)) +
      ((By-Ay)*(By-Ay))
    );
  
    double o = Math.atan(
      (Bx-Ax)/(By-Ax)
    );

    double n = Math.cos(
      90 - o
    );

    out[0] = Math.sqrt(
      (s*s)
      -((n-theta)*(n-theta))
    );
    
    out[1] = Math.sqrt(
      (s*s) + (out[0]*out[0])
    );

    return out;
  }

  public double[] Ratio(double[] points, double width) {
    double[] out = {0.0,0.0};
    double px = 0, py = 0;
    px = (1/2)*(points[0]);
    py = (1/2)*(points[1]);
    double c = px*(points[0]/points[1])+py;
    out[0] = c-(1/2)*width;
    out[1] = c+(1/2)*width;
    return out;
  }

  public double[] RobotDrivingRatio(double goalX, double goalY) { // TODO ITS NOT 10 BUT WHATEVER
    return Ratio(rPoints(Xposition, Yposition, goalX, goalY, gyroAngle), 6.8);
  }

  public double DistanceFromPoint(double Ax, double Ay, double Bx, double By) {
    double[] out = {0,0};
    out[0] = Math.abs(Bx - Ax);
    out[1] = Math.abs(By - Ay); // Ayo da pizza here
    return Math.sqrt((out[0]*out[0])+(out[1]+out[1]));
  } 
}
