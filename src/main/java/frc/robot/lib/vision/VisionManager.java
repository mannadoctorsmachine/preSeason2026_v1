package frc.robot.lib.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;

/**
 * Camada intermediaria entre as cameras (Limelight) e o resto do codigo: aceita QUANTAS cameras
 * quiser, filtra medicoes ruins, calcula a confianca (desvio padrao) e aplica a prioridade.
 *
 * <p>Regra de prioridade (modo {@link Mode#PRIORITY}): cameras sao agrupadas pelo numero de
 * prioridade (0 = principal). So o grupo mais prioritario que ainda tem visao envia dados; os
 * grupos de menor prioridade ficam em espera ate que TODOS os mais prioritarios percam as tags por
 * mais de {@code lossTimeoutSeconds}. Cameras com o mesmo numero de prioridade contribuem juntas.
 */
public class VisionManager {

  public enum Mode {
    /** Usa so o grupo de maior prioridade que ainda enxerga tags (fallback). */
    PRIORITY,
    /** Usa todas as cameras ao mesmo tempo (ignora prioridade). */
    FUSE_ALL
  }

  /** Estado de uma camera, para dashboard e logging. */
  public static final class CameraStatus {
    public final String name;
    public final int priority;
    public boolean connected;
    public boolean enabled;
    /** Viu tags validas recentemente (dentro do timeout de perda). */
    public boolean hasVision;
    /** Suas medicoes estao sendo enviadas ao estimador neste ciclo. */
    public boolean active;
    public int tagCount;
    public double avgTagDistanceMeters;
    public String rejectReason = "";

    CameraStatus(String name, int priority) {
      this.name = name;
      this.priority = priority;
    }
  }

  private static final class Entry {
    final VisionCamera camera;
    final VisionInputs inputs = new VisionInputs();
    final CameraStatus status;
    double lastValidTime = Double.NEGATIVE_INFINITY;
    VisionMeasurement pending = null;

    Entry(VisionCamera camera) {
      this.camera = camera;
      this.status = new CameraStatus(camera.getConfig().name, camera.getConfig().priority);
    }
  }

  private final List<Entry> entries = new ArrayList<>();
  private final double fieldLength;
  private final double fieldWidth;

  private Mode mode = Mode.PRIORITY;
  private double lossTimeoutSeconds = 0.25;
  private double maxYawRateRadPerSec = Math.toRadians(720.0);

  public VisionManager(double fieldLengthMeters, double fieldWidthMeters) {
    this.fieldLength = fieldLengthMeters;
    this.fieldWidth = fieldWidthMeters;
  }

  // ---------------------------------------------------------------------------------------------
  // Configuracao
  // ---------------------------------------------------------------------------------------------

  /** Adiciona uma camera (quantas quiser). @return this. */
  public VisionManager addCamera(VisionCamera camera) {
    entries.add(new Entry(camera));
    entries.sort(Comparator.comparingInt(e -> e.camera.getConfig().priority));
    return this;
  }

  public VisionManager setMode(Mode mode) {
    this.mode = mode;
    return this;
  }

  /** Tempo sem ver tags validas para uma camera ser considerada "perdida" (evita piscar). */
  public VisionManager setLossTimeout(double seconds) {
    this.lossTimeoutSeconds = seconds;
    return this;
  }

  /** Acima desta velocidade angular o robo esta girando rapido demais e a visao e ignorada. */
  public VisionManager setMaxYawRate(double radPerSec) {
    this.maxYawRateRadPerSec = radPerSec;
    return this;
  }

  /** Liga/desliga uma camera pelo nome. */
  public void setCameraEnabled(String name, boolean enabled) {
    for (Entry e : entries) {
      if (e.camera.getConfig().name.equals(name)) {
        e.camera.getConfig().enabled = enabled;
      }
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Ciclo
  // ---------------------------------------------------------------------------------------------

  /**
   * Le todas as cameras, filtra e devolve as medicoes que devem ir para o estimador de pose.
   * Chamar uma vez por ciclo.
   *
   * @param robotYaw Heading atual do robo (para MegaTag 2).
   * @param yawRateRadPerSec Velocidade angular atual do robo.
   */
  public List<VisionMeasurement> update(Rotation2d robotYaw, double yawRateRadPerSec) {
    double now = Timer.getFPGATimestamp();

    // 1) Leitura + filtros de qualidade de cada camera
    for (Entry e : entries) {
      VisionCameraConfig cfg = e.camera.getConfig();
      e.pending = null;
      e.status.rejectReason = "";
      e.status.active = false;
      e.status.enabled = cfg.enabled;

      e.camera.setRobotOrientation(robotYaw, yawRateRadPerSec);
      e.camera.updateInputs(e.inputs);
      Logger.processInputs("Vision/" + cfg.name, e.inputs);

      e.status.connected = e.inputs.connected;
      e.status.tagCount = e.inputs.tagCount;
      e.status.avgTagDistanceMeters = e.inputs.avgTagDistanceMeters;

      if (!cfg.enabled) {
        e.status.rejectReason = "desabilitada";
      } else if (!e.inputs.connected) {
        e.status.rejectReason = "desconectada";
      } else if (!e.inputs.hasEstimate) {
        e.status.rejectReason = "sem tags";
      } else {
        String reason = validate(cfg, e.inputs, yawRateRadPerSec);
        if (reason == null) {
          e.lastValidTime = now;
          e.pending = buildMeasurement(cfg, e.inputs);
        } else {
          e.status.rejectReason = reason;
        }
      }
      e.status.hasVision = (now - e.lastValidTime) <= lossTimeoutSeconds;
    }

    // 2) Aplica a prioridade
    List<VisionMeasurement> result = new ArrayList<>();
    if (mode == Mode.FUSE_ALL) {
      for (Entry e : entries) {
        if (e.pending != null) {
          result.add(e.pending);
          e.status.active = true;
        }
      }
      return result;
    }

    // Agrupa por prioridade (ordem crescente) e escolhe o primeiro grupo que ainda tem visao
    Map<Integer, List<Entry>> groups = new TreeMap<>();
    for (Entry e : entries) {
      groups.computeIfAbsent(e.camera.getConfig().priority, k -> new ArrayList<>()).add(e);
    }
    for (List<Entry> group : groups.values()) {
      boolean groupHasVision = group.stream().anyMatch(e -> e.status.hasVision);
      if (!groupHasVision) {
        continue; // grupo perdeu as tags: passa para o proximo (menor prioridade)
      }
      for (Entry e : group) {
        if (e.pending != null) {
          result.add(e.pending);
          e.status.active = true;
        }
      }
      break; // grupos de menor prioridade ficam em espera
    }
    return result;
  }

  // ---------------------------------------------------------------------------------------------
  // Consultas
  // ---------------------------------------------------------------------------------------------

  /** @return Estado de todas as cameras, da mais para a menos prioritaria. */
  public List<CameraStatus> getStatuses() {
    List<CameraStatus> list = new ArrayList<>();
    for (Entry e : entries) {
      list.add(e.status);
    }
    return list;
  }

  /** @return true se alguma camera habilitada viu tags validas recentemente. */
  public boolean hasAnyVision() {
    return entries.stream().anyMatch(e -> e.status.hasVision);
  }

  /** @return Nomes das cameras cujas medicoes estao sendo usadas agora. */
  public List<String> getActiveCameraNames() {
    List<String> names = new ArrayList<>();
    for (Entry e : entries) {
      if (e.status.active) {
        names.add(e.status.name);
      }
    }
    return names;
  }

  // ---------------------------------------------------------------------------------------------
  // Internos
  // ---------------------------------------------------------------------------------------------

  /** @return Motivo da rejeicao, ou null se a medicao e aceitavel. */
  private String validate(VisionCameraConfig cfg, VisionInputs in, double yawRateRadPerSec) {
    if (in.tagCount < cfg.minTagCount) {
      return "poucas tags";
    }
    if (in.avgTagDistanceMeters > cfg.maxAvgTagDistanceMeters) {
      return "tags longe demais";
    }
    if (cfg.useMegaTag2 && Math.abs(yawRateRadPerSec) > maxYawRateRadPerSec) {
      return "girando rapido demais";
    }
    Pose2d p = in.pose;
    if (p.getX() < 0.0 || p.getX() > fieldLength || p.getY() < 0.0 || p.getY() > fieldWidth) {
      return "pose fora do campo";
    }
    if (Double.isNaN(p.getX()) || Double.isNaN(p.getY())) {
      return "pose invalida";
    }
    return null;
  }

  private VisionMeasurement buildMeasurement(VisionCameraConfig cfg, VisionInputs in) {
    // Confianca: piora com o quadrado da distancia, melhora com mais tags
    double xy =
        cfg.xyStdDevBase
            * in.avgTagDistanceMeters
            * in.avgTagDistanceMeters
            / Math.max(1, in.tagCount)
            * cfg.stdDevScale;
    return new VisionMeasurement(
        cfg.name,
        cfg.priority,
        in.pose,
        in.timestampSeconds,
        in.tagCount,
        in.avgTagDistanceMeters,
        VecBuilder.fill(xy, xy, cfg.thetaStdDevRad));
  }
}
