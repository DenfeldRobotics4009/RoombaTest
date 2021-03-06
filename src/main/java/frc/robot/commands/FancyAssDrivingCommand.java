// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.ser.std.StdArraySerializers.DoubleArraySerializer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class FancyAssDrivingCommand extends CommandBase {
  private DriveTrain Drive;
  /** Creates a new FancyAssDrivingCommand. */
  public FancyAssDrivingCommand(DriveTrain drive) {
    Drive = drive;  
    addRequirements(Drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double[] DrivingSpeeds = Drive.RobotDrivingRatio(10, 10); 

    if ((Drive.DistanceFromPoint(Drive.Xposition, Drive.Yposition, 10.0, 10.0)) > 2){
      Drive.drive(DrivingSpeeds);  
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
