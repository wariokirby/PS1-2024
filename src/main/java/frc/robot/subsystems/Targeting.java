// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Targeting extends SubsystemBase {
  private final double HEIGHT_OF_TARGET = 8.67; //TODO need real height
  private final double HEIGHT_OF_CAMERA = 37.5 / 12;//TODO need real height
  private final double MOUNT_ANGLE = 33.1; //TODO need angle

  private NetworkTable limelight;
  private NetworkTableEntry tv;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;

  private double validTarget;
  private double x;
  private double y;
  private double a;

  /** Creates a new Targeting. */
  public Targeting() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    tv = limelight.getEntry("tv");
    tx = limelight.getEntry("tx");
    ty = limelight.getEntry("ty");
    ta = limelight.getEntry("ta");

    validTarget = 0;
    x = 100;
    y = 100;
    a = -1;

    limelight.getEntry("ledMode").setNumber(3);
    limelight.getEntry("camMode").setNumber(0);
    limelight.getEntry("pipeline").setNumber(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    validTarget = tv.getDouble(0);
    x = tx.getDouble(0);
    y = ty.getDouble(0);
    a = ta.getDouble(0);

    SmartDashboard.putNumber("Limelight Valid Target", validTarget);
    SmartDashboard.putNumber("limelight x", x);
    SmartDashboard.putNumber("limelight y", y);
    //SmartDashboard.putNumber("limelight a", a);
    SmartDashboard.putNumber("range", calcRange());
  }

  public double getValidTarget(){
    return validTarget;
  }
  
  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getA() {
    return a;
  }

  public double calcRange() {
    double d = (HEIGHT_OF_TARGET - HEIGHT_OF_CAMERA) / Math.tan(Math.toRadians(MOUNT_ANGLE + y));
    return d + 2.5;       
  }

}