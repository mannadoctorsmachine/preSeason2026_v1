package frc.robot.lib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Medicao de visao ja filtrada e pronta para o Pose Estimator.
 * @param cameraName Camera que originou a medicao.
 * @param priority Prioridade da camera (0 = principal).
 * @param pose Pose do robo no campo.
 * @param timestampSeconds Instante da captura (base de tempo do FPGA).
 * @param stdDevs Desvio padrao [x (m), y (m), theta (rad)].
 */
public record VisionMeasurement(
    String cameraName,
    int priority,
    Pose2d pose,
    double timestampSeconds,
    int tagCount,
    double avgTagDistanceMeters,
    Matrix<N3, N1> stdDevs) {}
