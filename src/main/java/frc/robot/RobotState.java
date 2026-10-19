package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Fonte unica da verdade sobre ONDE o robo esta no campo (camada de odometria).
 *
 * <p>Funde a odometria das rodas + giroscopio (enviada pelo SwerveSubsystem) com as medicoes de visao
 * (enviadas pelo VisionSubsystem) usando um Pose Estimator. Nenhum dos dois subsistemas conhece o
 * outro: ambos apenas alimentam este objeto, e quem precisa da pose (ex: GoTo, PathPlanner) le daqui.
 *
 * <p>Convencao: origem do campo no canto do lado AZUL, +X em direcao ao lado vermelho, +Y para a
 * esquerda, angulos positivos anti-horarios. A pose NUNCA e espelhada pela alianca: quem precisa
 * (joystick, destinos, trajetorias) converte para o referencial do operador.
 */
public class RobotState {
  /** Confianca padrao na odometria das rodas [x (m), y (m), theta (rad)] - maior = confia menos. */
  private static final Matrix<N3, N1> kDefaultStateStdDevs = VecBuilder.fill(0.05, 0.05, 0.02);

  private final SwerveDrivePoseEstimator poseEstimator;

  /** Odometria pura (sem visao), mantida so para depuracao: compare com a pose fundida no log. */
  private final SwerveDriveOdometry odometryOnly;

  private Rotation2d lastGyroYaw = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = zeroPositions();
  private ChassisSpeeds robotRelativeVelocity = new ChassisSpeeds();

  private int acceptedVisionMeasurements = 0;
  private double lastVisionTimestamp = Double.NEGATIVE_INFINITY;

  public RobotState(SwerveDriveKinematics kinematics) {
    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.kZero,
            zeroPositions(),
            Pose2d.kZero,
            kDefaultStateStdDevs,
            VecBuilder.fill(0.5, 0.5, 1.0e6));
    this.odometryOnly =
        new SwerveDriveOdometry(kinematics, Rotation2d.kZero, zeroPositions(), Pose2d.kZero);
  }

  private static SwerveModulePosition[] zeroPositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };
  }

  // ---------------------------------------------------------------------------------------------
  // Entradas
  // ---------------------------------------------------------------------------------------------

  /** Odometria: chamada pelo SwerveSubsystem a cada ciclo. */
  public void addOdometryObservation(
      double timestampSeconds, Rotation2d gyroYaw, SwerveModulePosition[] modulePositions) {
    lastGyroYaw = gyroYaw;
    lastModulePositions = modulePositions;
    poseEstimator.updateWithTime(timestampSeconds, gyroYaw, modulePositions);
    odometryOnly.update(gyroYaw, modulePositions);
  }

  /**
   * Visao: chamada pelo VisionSubsystem quando uma AprilTag e vista.
   * @param visionPose Pose do robo estimada pela camera (referencial do campo).
   * @param timestampSeconds Instante da captura (mesma base de tempo do FPGA).
   * @param stdDevs Desvio padrao [x (m), y (m), theta (rad)]; maior = menos confianca.
   */
  public void addVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
    acceptedVisionMeasurements++;
    lastVisionTimestamp = timestampSeconds;
  }

  /** Velocidade medida do chassi (referencial do robo), informada pelo SwerveSubsystem. */
  public void setRobotRelativeVelocity(ChassisSpeeds speeds) {
    this.robotRelativeVelocity = speeds;
  }

  // ---------------------------------------------------------------------------------------------
  // Saidas
  // ---------------------------------------------------------------------------------------------

  /** @return Pose estimada do robo no campo (referencial azul, sem espelhamento). */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** @return Pose estimada em um instante passado (ex: para compensar latencia), se houver historico. */
  public Optional<Pose2d> getPoseAt(double timestampSeconds) {
    return poseEstimator.sampleAt(timestampSeconds);
  }

  /** @return Pose so da odometria (sem visao). Util para depurar o quanto a visao corrige. */
  public Pose2d getOdometryOnlyPose() {
    return odometryOnly.getPoseMeters();
  }

  /** @return Orientacao estimada do robo no campo. */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /** @return Velocidade do robo no referencial do robo. */
  public ChassisSpeeds getRobotRelativeVelocity() {
    return robotRelativeVelocity;
  }

  /** @return Velocidade do robo no referencial do campo. */
  public ChassisSpeeds getFieldRelativeVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, getHeading());
  }

  /** @return true se a visao corrigiu a pose nos ultimos {@code withinSeconds} segundos. */
  public boolean hasRecentVisionFix(double withinSeconds) {
    return Timer.getFPGATimestamp() - lastVisionTimestamp <= withinSeconds;
  }

  /** @return Distancia (m) entre a pose fundida e a odometria pura (quanto a visao ja corrigiu). */
  public double getVisionCorrectionMeters() {
    return getPose().getTranslation().getDistance(getOdometryOnlyPose().getTranslation());
  }

  // ---------------------------------------------------------------------------------------------
  // Reset
  // ---------------------------------------------------------------------------------------------

  /** Redefine a pose (ex: posicao inicial do autonomo) usando as ultimas leituras de gyro e rodas. */
  public void resetPose(Pose2d pose) {
    resetPose(pose, lastGyroYaw, lastModulePositions);
  }

  /** Redefine a pose com leituras de gyro e rodas explicitas (usado na inicializacao). */
  public void resetPose(Pose2d pose, Rotation2d gyroYaw, SwerveModulePosition[] modulePositions) {
    lastGyroYaw = gyroYaw;
    lastModulePositions = modulePositions;
    poseEstimator.resetPosition(gyroYaw, modulePositions, pose);
    odometryOnly.resetPosition(gyroYaw, modulePositions, pose);
  }

  // ---------------------------------------------------------------------------------------------
  // Logging
  // ---------------------------------------------------------------------------------------------

  /** Registra o estado no AdvantageKit. Chamar uma vez por ciclo (Robot.robotPeriodic). */
  public void log() {
    Logger.recordOutput("RobotState/Pose", getPose());
    Logger.recordOutput("RobotState/OdometryOnlyPose", getOdometryOnlyPose());
    Logger.recordOutput("RobotState/VisionCorrectionMeters", getVisionCorrectionMeters());
    Logger.recordOutput("RobotState/FieldVelocity", getFieldRelativeVelocity());
    Logger.recordOutput("RobotState/AcceptedVisionMeasurements", acceptedVisionMeasurements);
    Logger.recordOutput("RobotState/HasRecentVisionFix", hasRecentVisionFix(1.0));
  }
}
