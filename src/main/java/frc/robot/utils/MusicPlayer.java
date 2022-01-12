package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

public class MusicPlayer {

  private static MusicPlayer instance = null;

  public static synchronized MusicPlayer getInstance() {
    if (instance == null) instance = new MusicPlayer();
    return instance;
  }

  private static final String[] playlist = {
    "AMOGUS.chrp", "MEGLOV.chrp", "BFIFTH.chrp", "AVNGER.chrp", "IMMRCH.chrp", "STLALV.chrp"
  };

  private Orchestra orch = new Orchestra();

  public void addMotor(TalonFX... talon) {
    for (TalonFX motor : talon) orch.addInstrument(motor);
  }

  public void play(String file) {
    orch.loadMusic(file);
    orch.play();
  }

  public void play() {
    play(playlist[(int) Math.random() * playlist.length]);
  }

  public boolean isPlaying() {
    return orch.isPlaying();
  }

  public void stop() {
    orch.stop();
  }
}
