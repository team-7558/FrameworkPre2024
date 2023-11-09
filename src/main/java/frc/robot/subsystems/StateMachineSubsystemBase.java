// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public abstract class StateMachineSubsystemBase extends SubsystemBase {

  protected abstract class State {
    public final String name;
    private boolean isActive;
    private Timer timer;
    private StateMachineSubsystemBase subsystem;

    public State(String name) {
      this.subsystem = StateMachineSubsystemBase.this;
      this.name = name;
      isActive = false;
      timer = new Timer();
    }

    // Methods to override
    public void init() {}

    public void periodic() {}

    public void exit() {}

    public final void baseInit() {
      init();
      timer.restart();
      isActive = true;
    }

    public final void basePeriodic() {
      periodic();
    }

    public final void baseExit() {
      exit();
      timer.stop();
      isActive = false;
    }

    public double time() {
      if (isActive) {
        return timer.get();
      } else {
        return -1.0;
      }
    }

    public void reset() {
      if (isActive) {
        timer.reset();
      } else {
      }
    }

    public boolean after(double seconds) {
      if (isActive) {
        return timer.hasElapsed(seconds);
      } else {
        return false;
      }
    }

    public boolean before(double seconds) {
      if (isActive) {
        return !timer.hasElapsed(seconds);
      } else {
        return true;
      }
    }

    public boolean between(double earlier, double later) {
      if (isActive) {
        double t = timer.get();
        return earlier < t && t < later;
      } else {
        return false;
      }
    }

    public double alpha(double earlier, double later) {
      if (isActive) {
        double t = timer.get();
        if (t <= earlier) return 0.0;
        if (t >= later) return 1.0;
        return Util.unlerp(earlier, later, t);
      } else {
        return 0.0;
      }
    }

    public boolean isActive() {
      return isActive;
    }

    @Override
    public boolean equals(Object obj) {
      if (obj == null) {
        return false;
      }

      if (obj.getClass() != this.getClass()) {
        return false;
      }

      final State other = (State) obj;
      if ((this.name == null) ? (other.name != null) : !this.name.equals(other.name)) {
        return false;
      }

      return true;
    }

    @Override
    public String toString() {
      return name;
    }
  }

  private State currentState;

  public State getCurrentState() {
    return currentState;
  }

  public void setCurrentState(State state) {
    if (state.subsystem != this) {
      System.err.printf(
          "[SME] Cannot set the state of <%s> to <%s>.<%s>\n",
          this.getName(), state.subsystem.getName(), state.name);
    }
    if (currentState == null) {
      currentState = state;
      currentState.baseInit();
    } else if (!currentState.equals(state)) {
      currentState.baseExit();
      currentState = state;
      currentState.baseInit();
    }
  }

  public boolean isState(String state) {
    return currentState.name.equals(state);
  }

  public boolean isState(State state) {
    return currentState.equals(state);
  }

  /** Creates a new StateMachineSubsystem. */
  public StateMachineSubsystemBase(String name) {
    this.setName(name);
  }

  public abstract void outputPeriodic();

  public void inputPeriodic() {}

  @Override
  public final void periodic() {
    long time = Logger.getRealTimestamp();
    inputPeriodic();
    currentState.basePeriodic();
    outputPeriodic();
    Logger.recordOutput(this.getName() + "/PerfDelta", Logger.getRealTimestamp() - time);
    Logger.recordOutput(this.getName() + "/State", currentState.name);
  }

  public final void controlsPeriodic() {}
}
