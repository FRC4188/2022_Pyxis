// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class CSPController extends XboxController {
  

  private final class Buttons {
    static final int A = 1;
    static final int B = 2;
    static final int X = 3;
    static final int Y = 4;
    static final int LB = 5;
    static final int RB = 6;
    static final int SQUARES = 7;
    static final int LINES = 8;
    static final int RS = 9;
    static final int LS = 10;

    static final int DPAD_UP = 0;
    static final int DPAD_RIGHT = 90;
    static final int DPAD_DOWN = 180;
    static final int DPAD_LEFT = 270;
  }

  public enum Scaling {
    LINEAR,
    SQUARED,
    CUBED
  }

  /** Creates a new CSPController. */
  public CSPController(int port) {
    super(port);
  }

  /** Converts joystick input to output based on scaling and deadband. */
  private double getOutput(double input, Scaling scaling) {
    if (Math.abs(input) > Constants.controller.DEADBAND) {
      if (scaling == Scaling.SQUARED) return Math.signum(input) * Math.pow(input, 2);
      else if (scaling == Scaling.CUBED) return Math.pow(input, 3);
      else return input;
    } else {
      return 0;
    }
  }

  public double getRightX(Scaling scaling) {
    return getOutput(getRightX(), scaling);
  }

  public double getRightY(Scaling scaling) {
    return -getOutput(getRightY(), scaling);
  }

  public double getLeftX(Scaling scaling) {
    return getOutput(getLeftX(), scaling);
  }

  public double getLeftY(Scaling scaling) {
    return -getOutput(getLeftY(), scaling);
  }
  /** Returns the JoystickButton object for the A button. */
  public JoystickButton getAButtonObj() {
    return new JoystickButton(this, Buttons.A);
  }

  /** Returns the JoystickButton object for the B button. */
  public JoystickButton getBButtonObj() {
    return new JoystickButton(this, Buttons.B);
  }

  /** Returns the JoystickButton object for the X button. */
  public JoystickButton getXButtonObj() {
    return new JoystickButton(this, Buttons.X);
  }

  /** Returns the JoystickButton object for the Y button. */
  public JoystickButton getYButtonObj() {
    return new JoystickButton(this, Buttons.Y);
  }

  /** Returns the JoystickButton object for the left bumper button. */
  public JoystickButton getLbButtonObj() {
    return new JoystickButton(this, Buttons.LB);
  }

  /** Returns the JoystickButton object for the right bumper button. */
  public JoystickButton getRbButtonObj() {
    return new JoystickButton(this, Buttons.RB);
  }

  /** Returns the JoystickButton object for the back button. */
  public JoystickButton getSquaresButtonObj() {
    return new JoystickButton(this, Buttons.SQUARES);
  }

  /** Returns the JoystickButton object for the start button. */
  public JoystickButton getLinesButtonObj() {
    return new JoystickButton(this, Buttons.LINES);
  }

  /** Returns the JoystickButton object for the left stick button. */
  public JoystickButton getLsButtonObj() {
    return new JoystickButton(this, Buttons.LS);
  }

  /** Returns the JoystickButton object for the right stick button. */
  public JoystickButton getRsButtonObj() {
    return new JoystickButton(this, Buttons.RS);
  }

  /** Returns the POVButton object for the D-pad up button. */
  public POVButton getDpadUpButtonObj() {
    return new POVButton(this, Buttons.DPAD_UP);
  }

  /** Returns the POVButton object for the D-pad right button. */
  public POVButton getDpadRightButtonObj() {
    return new POVButton(this, Buttons.DPAD_RIGHT);
  }

  /** Returns the POVButton object for the D-pad down button. */
  public POVButton getDpadDownButtonObj() {
    return new POVButton(this, Buttons.DPAD_DOWN);
  }

  /** Returns the POVButton object for the D-pad left button. */
  public POVButton getDpadLeftButtonObj() {
    return new POVButton(this, Buttons.DPAD_LEFT);
  }

  /** Returns the Trigger object for the left trigger. */
  public Trigger getLtButtonObj() {
    return new Trigger(() -> getLeftTriggerAxis() > Constants.controller.TRIGGER_THREADSHOLD);
  }

  /** Returns the Trigger object for the right trigger. */
  public Trigger getRtButtonObj() {
    return new Trigger(() -> getRightTriggerAxis() > Constants.controller.TRIGGER_THREADSHOLD);
  }

  public void setRRumble(double intensity) {
    setRumble(RumbleType.kRightRumble, intensity);
  }

  public void setLRumble(double intensity) {
    setRumble(RumbleType.kLeftRumble, intensity);
  }

  public void setRumble(double intensity) {
    setRRumble(intensity);
    setLRumble(intensity);
  }
 }
