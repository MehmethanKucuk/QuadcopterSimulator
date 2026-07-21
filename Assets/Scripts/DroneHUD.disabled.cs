using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.UI;

public class DroneHUD : MonoBehaviour
{
    [Header("Referanslar")]
    public Text statusText;    // üst kutu (kısa bilgi)
    public Text historyText;   // alt kutu (PID + kontroller + history)

    [Header("Opsiyonel script referansları (mod isimleri / kazançlar için)")]
    public DroneController droneController;
    public DroneCameraController cameraController;

    // SimulationManager burayı her görselleştirme adımında çağırıyor
    public void UpdateHUD(DroneState state, List<DroneSample> history)
    {
        if (statusText != null)
            statusText.text = BuildStatusString(state);

        if (historyText != null)
            historyText.text = BuildInfoAndHistoryString(history);
    }

    // ------------------------------------------------------------------
    // ÜST KUTU: DRONE STATE + MODLAR (KISA)
    // ------------------------------------------------------------------
    private string BuildStatusString(DroneState state)
    {
        if (state == null)
            return "No state.";

        StringBuilder sb = new StringBuilder();

        sb.AppendLine("==== DRONE ====");
        sb.AppendLine($"Pos: x={state.position.x,5:0.00}  y={state.position.y,5:0.00}  z={state.position.z,5:0.00}");
        sb.AppendLine($"Vel: vx={state.velocity.x,5:0.00} vy={state.velocity.y,5:0.00} vz={state.velocity.z,5:0.00}");
        sb.AppendLine($"Ang: p={state.eulerAngles.x,5:0.0}  y={state.eulerAngles.y,5:0.0}  r={state.eulerAngles.z,5:0.0}");
        sb.AppendLine($"W  : wx={state.angularVelocity.x,5:0.0} wy={state.angularVelocity.y,5:0.0} wz={state.angularVelocity.z,5:0.0}");
        sb.AppendLine();

        string controlMode = (droneController != null) ? droneController.controlMode.ToString() : "N/A";
        string camMode     = (cameraController != null) ? cameraController.mode.ToString() : "N/A";

        sb.AppendLine($"Ctrl : {controlMode}   (TAB)");
        sb.AppendLine($"Cam  : {camMode}       (C)");

        return sb.ToString();
    }

    // ------------------------------------------------------------------
    // ALT KUTU: PID + KONTROLLER + LAST 10 SAMPLES
    // ------------------------------------------------------------------
    private string BuildInfoAndHistoryString(List<DroneSample> history)
    {
        StringBuilder sb = new StringBuilder();

        // PID GAINS
        if (droneController != null)
        {
            sb.AppendLine("==== PID ====");
            sb.AppendLine($"Pitch: Kp={droneController.pitchKp:0.0} Ki={droneController.pitchKi:0.0} Kd={droneController.pitchKd:0.0}");
            sb.AppendLine($"Roll : Kp={droneController.rollKp:0.0} Ki={droneController.rollKi:0.0} Kd={droneController.rollKd:0.0}");
            sb.AppendLine($"Yaw  : Kp={droneController.yawKp:0.0} Ki={droneController.yawKi:0.0} Kd={droneController.yawKd:0.0}");
            sb.AppendLine($"Alt  : Kp={droneController.altKp:0.0} Ki={droneController.altKi:0.0} Kd={droneController.altKd:0.0}");
            sb.AppendLine();
        }

        // CONTROLS
        sb.AppendLine("==== CONTROLS ====");
        sb.AppendLine("Move : W/S pitch, A/D roll");
        sb.AppendLine("Yaw  : Q/E");
        sb.AppendLine("Alt  : Space / LCtrl");
        sb.AppendLine("Modes: TAB ctrl, C cam");
        sb.AppendLine("Target: Mouse L-Click (AutoPID)");
        sb.AppendLine();

        // HISTORY TABLE
        if (history == null || history.Count == 0)
        {
            sb.AppendLine("No history yet.");
        }
        else
        {
            sb.AppendLine("==== LAST SAMPLES ====");
            sb.AppendLine(" t[s]   x    y    z  |  vx   vy   vz");

            foreach (var h in history)
            {
                sb.AppendLine(
                    $"{h.time,5:0.0} {h.position.x,4:0.0} {h.position.y,4:0.0} {h.position.z,4:0.0} | " +
                    $"{h.velocity.x,4:0.0} {h.velocity.y,4:0.0} {h.velocity.z,4:0.0}"
                );
            }
        }

        return sb.ToString();
    }
}
