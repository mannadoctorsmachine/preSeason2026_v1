// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.navigation.GoToPoseCommand;
import frc.robot.lib.navigation.ObstacleField;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Fabrica de comandos autonomos. Mistura os dois modos de navegacao do robo:
 * <ul>
 *   <li>PathPlanner: trajetorias desenhadas na GUI (autos e paths em src/main/deploy/pathplanner).
 *   <li>GoTo: ir a uma coordenada qualquer desviando das zonas restritas (lib.navigation).
 * </ul>
 * Tudo e definido no referencial AZUL; o espelhamento para a alianca vermelha e automatico.
 */
public final class Autos {
  /** Restricoes padrao para pathfinding do PathPlanner (m/s, m/s^2, rad/s, rad/s^2). */
  public static final PathConstraints kDefaultConstraints =
      new PathConstraints(3.0, 3.0, Math.toRadians(540.0), Math.toRadians(720.0));

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Registra comandos que podem ser usados como "Named Commands" nos autos da GUI do PathPlanner.
   * Chamar ANTES de criar o seletor de autonomo.
   */
  public static void registerNamedCommands(
      SwerveSubsystem swerve, ObstacleField obstacles) {
    NamedCommands.registerCommand("Stop", Commands.runOnce(swerve::stop, swerve));
    NamedCommands.registerCommand("GoToCenter", goTo(swerve, obstacles, new Translation2d(8.27, 4.03)));
    //NamedCommands.registerCommand("IntakeIn", intake.runPercentCommand(frc.robot.Constants.IntakeConstants.kIntakeSpeed));
    //NamedCommands.registerCommand("IntakeOut", intake.runPercentCommand(frc.robot.Constants.IntakeConstants.kOuttakeSpeed));
    //NamedCommands.registerCommand("IntakeStop", intake.stopCommand());
  }

  /** Segue uma trajetoria criada na GUI do PathPlanner (nome do arquivo .path, sem extensao). */
  public static Command followPath(String pathName) {
    try {
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
    } catch (Exception e) {
      DriverStation.reportError("Falha ao carregar path '" + pathName + "': " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  /** Planeja uma rota ate a pose com o pathfinder do PathPlanner (usa o mapa navgrid da GUI). */
  public static Command pathfindTo(Pose2d targetBlue) {
    return AutoBuilder.pathfindToPoseFlipped(targetBlue, kDefaultConstraints);
  }

  /** Vai ate a coordenada desviando das zonas restritas, mantendo o heading atual. */
  public static Command goTo(SwerveSubsystem swerve, ObstacleField obstacles, Translation2d target) {
    return new GoToPoseCommand(swerve, obstacles, target);
  }

  /** Vai ate a pose desviando das zonas restritas e gira para o heading do destino. */
  public static Command goTo(SwerveSubsystem swerve, ObstacleField obstacles, Pose2d target) {
    return new GoToPoseCommand(swerve, obstacles, target);
  }

  /** Autonomo de exemplo: sai do lugar ate um ponto, desviando dos obstaculos. */
  public static Command exampleAuto(SwerveSubsystem swerve, ObstacleField obstacles) {
    return Commands.sequence(
        goTo(swerve, obstacles, new Translation2d(5.0, 4.0)).withTimeout(8.0),
        Commands.runOnce(swerve::stop, swerve));
  }
}
