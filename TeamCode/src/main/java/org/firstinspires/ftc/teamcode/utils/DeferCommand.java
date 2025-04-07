package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.PrintCommand;
import java.util.Set;
import java.util.function.Supplier;

public class DeferCommand extends CommandBase {
  private final Command m_nullCommand =
      new PrintCommand("[DeferredCommand] Supplied command was null!");

  private final Supplier<Command> m_supplier;
  private Command m_command = m_nullCommand;

  /**
   * Creates a new DeferredCommand that directly runs the supplied command when initialized, and
   * ends when it ends. Useful for lazily creating commands when the DeferredCommand is initialized,
   * such as if the supplied command depends on runtime state. The {@link Supplier} will be called
   * each time this command is initialized. The Supplier <i>must</i> create a new Command each call.
   *
   * @param supplier The command supplier
   * @param requirements The command requirements. This is a {@link Set} to prevent accidental
   *     omission of command requirements. Use {@link Set#of()} to easily construct a requirement
   *     set.
   */
  @SuppressWarnings("this-escape")
  public DeferCommand(Supplier<Command> supplier) {
    m_supplier = supplier;
  }

  @Override
  public void initialize() {
    Command cmd = m_supplier.get();
    if (cmd != null) {
      m_command = cmd;
      CommandScheduler.getInstance().schedule(m_command);
    }
    m_command.initialize();
  }

  @Override
  public void execute() {
    m_command.execute();
  }

  @Override
  public boolean isFinished() {
    return m_command.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    m_command.end(interrupted);
    m_command = m_nullCommand;
  }
}
