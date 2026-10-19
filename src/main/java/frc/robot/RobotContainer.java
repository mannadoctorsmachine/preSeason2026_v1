// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.auto.PathPlannerSetup;
import frc.robot.commands.Autos;
import frc.robot.commands.swerve.DriveModeSelector;
import frc.robot.commands.swerve.TeleopDriveInputs;
import frc.robot.lib.navigation.GoToPoseCommand;
import frc.robot.lib.navigation.ObstacleField;
import frc.robot.lib.vision.VisionCameraConfig;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Onde o robo e montado: subsistemas, camadas intermediarias, bindings e autonomos.
 *
 * <p>Fluxo de dados: Swerve (rodas + gyro) e Vision (Limelights) alimentam o {@link RobotState};
 * GoTo e PathPlanner leem a pose do RobotState e comandam o Swerve.
 *
 * <p>Dois controles: o do PILOTO (porta 0) controla somente o swerve; o do OPERADOR (porta 1)
 * controla os subsistemas (coleta, intake, shooter).
 */
public class RobotContainer {
  /** Piloto: controla somente o swerve. */
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** Operador: controla coleta, intake e shooter. */
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final SwerveSubsystem m_swerve = new SwerveSubsystem();
  private final RobotState m_robotState = new RobotState(m_swerve.getKinematics());

  /** Modo de direcao do piloto (FIELD ou FREEDOM). */
  private final DriveModeSelector m_driveMode = new DriveModeSelector();

  /** Coleta (rolo), pivo do intake e conjunto Index/Intermediario/Shooter. */
  private final CollectorSubsystem m_collector = new CollectorSubsystem();

  private final IntakePivotSubsystem m_intakePivot = new IntakePivotSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  /**
   * Visao: adicione quantas Limelights quiser. Prioridade 0 = principal; numeros maiores so enviam
   * dados quando as de prioridade menor perdem as AprilTags. Mesma prioridade = contribuem juntas.
   */
  @SuppressWarnings("unused") // so precisa existir: seu periodic() alimenta o RobotState
  private final VisionSubsystem m_vision =
      new VisionSubsystem(m_robotState)
          .addLimelight(new VisionCameraConfig("limelight-front").withPriority(0))
          .addLimelight(new VisionCameraConfig("limelight-back").withPriority(1));

  /**
   * Zonas restritas do campo (metros, referencial AZUL). Marque aqui os obstaculos; o raio do robo
   * ja e somado como folga e o espelhamento para a alianca vermelha e automatico.
   */
  private final ObstacleField m_obstacles =
      new ObstacleField(m_swerve.getCircumradiusMeters());

  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    // TODO: cadastrar obstaculos reais, ex: m_obstacles.addObstacle(x1, y1, x2, y2);

    PathPlannerSetup.configure(m_swerve, m_robotState);
    Autos.registerNamedCommands(m_swerve, m_obstacles);

    // Seletor: lista os autos criados na GUI do PathPlanner + um auto de exemplo com GoTo
    m_autoChooser = AutoBuilder.buildAutoChooser();
    m_autoChooser.addOption("GoTo Example", Autos.exampleAuto(m_swerve, m_obstacles));
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    configureBindings();
    configureOperatorBindings();
    m_driveMode.publish();
  }

  // PILOTO (porta 0): somente o swerve.
  //   Analogico esquerdo = direcao | Analogico direito = giro
  //   RT = acelerar | LT = trocar de modo (FIELD <-> FREEDOM) | START = zerar a frente
  private void configureBindings() {
    // -------------------------------------------------------------------------------------------
    // 1. ANDAR PARA TODOS OS LADOS (Swerve Drive) - dois modos, cada um e um comando separado:
    //    FIELD   (FieldDriveCommand):   a frente e sempre a de quando o robo ligou/zerou
    //    FREEDOM (FreedomDriveCommand): sem frente fixa, relativo ao robo
    // -------------------------------------------------------------------------------------------
    // Analógicos: Left Y (frente/trás), Left X (esquerda/direita), Right X (giro); RT acelera
    TeleopDriveInputs driveInputs =
        new TeleopDriveInputs(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX(),
            m_driverController::getRightTriggerAxis);
    m_swerve.setDefaultCommand(m_driveMode.createDefaultCommand(m_swerve, driveInputs));

    // Botão LT: troca o modo de direção; o controle vibra um instante para confirmar
    m_driverController
        .leftTrigger()
        .onTrue(
            Commands.runOnce(m_driveMode::toggle)
                .andThen(
                    Commands.startEnd(
                            () -> m_driverController.setRumble(RumbleType.kBothRumble, 0.7),
                            () -> m_driverController.setRumble(RumbleType.kBothRumble, 0.0))
                        .withTimeout(0.15)));

    // Botão START: zera a orientação do robô no campo (reset do Field-Oriented): a direção em que o
    // robô aponta agora vira a frente do modo FIELD
    m_driverController.start().onTrue(Commands.runOnce(m_swerve::zeroHeading));

    // Botão X: Trava as rodas em formato "X" (impede o robô de ser empurrado na defesa)
    m_driverController.x().whileTrue(Commands.run(m_swerve::lockWheels, m_swerve));

    // -------------------------------------------------------------------------------------------
    // 2. ANDAR PARA X COORDENADA (Navegação Autônoma em Teleop)
    // -------------------------------------------------------------------------------------------
    // Botão A: Segure para ir até a coordenada (X = 5.0m, Y = 4.0m) desviando de obstáculos
    m_driverController
        .a()
        .whileTrue(new GoToPoseCommand(m_swerve, m_obstacles, new Translation2d(5.0, 4.0)));

    // Botão B: Segure para ir até uma pose completa (X = 2.0m, Y = 1.5m) e virar a 180 graus
    m_driverController
        .b()
        .whileTrue(
            new GoToPoseCommand(
                m_swerve,
                m_obstacles,
                new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(180.0))));
  }

  // OPERADOR (porta 1): coleta, intake e shooter.
  //   LT (segurar) = coleta gira (anti-horario)
  //   LB = intake desce | RB = intake sobe (cliques repetidos sao ignorados)
  //   RT (segurar) = Index -> Intermediario -> Shooter
  private void configureOperatorBindings() {
    m_operatorController.leftTrigger().whileTrue(m_collector.collectCommand());

    m_operatorController.leftBumper().onTrue(m_intakePivot.lowerCommand());
    m_operatorController.rightBumper().onTrue(m_intakePivot.raiseCommand());

    m_operatorController.rightTrigger().whileTrue(m_shooter.shootCommand());
  }

  /** Registro de saidas que nao pertencem a nenhum subsistema. Chamado em Robot.robotPeriodic. */
  public void logPeriodic() {
    m_robotState.log();
    m_driveMode.publish();
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
