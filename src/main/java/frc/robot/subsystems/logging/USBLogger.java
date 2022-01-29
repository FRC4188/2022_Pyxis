// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.logging;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class USBLogger extends SubsystemBase {

  private static USBLogger instance = null;
  public static USBLogger getInstance() {
    if (instance == null) instance = new USBLogger();
    return instance;
  }

  private static final double start = System.currentTimeMillis() / 1000.0;

  ArrayList<Supplier<Data>> suppliers = new ArrayList<Supplier<Data>>();

  private File logFile;

  private Notifier logger;

  /** Creates a new USBLogger. */
  private USBLogger() {
    System.out.println(String.join(", ", File.listRoots().toString()));
    CommandScheduler.getInstance().registerSubsystem(this);

    try {
      logFile = new File("\\test\\log.txt");
      logFile.createNewFile();
      System.out.println(logFile.exists());
    } catch (Exception e) {
      DriverStation.reportError(e.getMessage(), false);
      System.out.println(e.getLocalizedMessage());
    }

    logger = new Notifier(() -> logSuppliers());
  }

  @Override
  public void periodic() {
  }

  public void setPeriod(double seconds) {
    logger.stop();
    logger.startPeriodic(seconds);
  }

  public void addSupplier(Supplier<Data> supplier) {
      suppliers.add(supplier);
  }

  public void logData(Data... datum) {
    try {
      BufferedWriter writer = new BufferedWriter(new FileWriter(logFile));
      writer.append(format(getTime(), datum));
      writer.close();
    } catch (Exception e) {
      DriverStation.reportError(e.getMessage(), false);
      System.out.println(e.getLocalizedMessage());
    }
  }

  private String format(double time, Data... datum) {
    StringBuilder builder = new StringBuilder();

    builder.append(time);
    for (Data data : datum) {
      builder.append(";");
      builder.append(data.getLabel());
      builder.append(":");
      builder.append(data.getValue());
    }
    builder.append("\n");

    return builder.toString();
  }

  private double getTime() {
    return (System.currentTimeMillis() / 1000.0) - start;
  }

  private void logSuppliers() {
    Data[] array = new Data[suppliers.size()];
    for (int i = 0; i < suppliers.size(); i++) array[i] = suppliers.get(i).get();

    logData(array);
  }
}
