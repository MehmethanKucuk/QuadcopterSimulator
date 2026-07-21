using System.Text;
using TMPro;
using UnityEngine;

public class DroneHistoryPanel : MonoBehaviour
{
    public SimulationManager sim;
    public TMP_Text historyText;

    void Start()
    {
        if (sim == null) sim = FindFirstObjectByType<SimulationManager>();
    }

    void Update()
    {
        if (sim == null || historyText == null) return;

        var list = sim.GetHistory();
        var s = sim.GetState();

        StringBuilder sb = new StringBuilder(512);

        sb.AppendLine($"POS: {s.position.x,6:0.00} {s.position.y,6:0.00} {s.position.z,6:0.00}");
        sb.AppendLine($"VEL: {s.velocity.x,6:0.00} {s.velocity.y,6:0.00} {s.velocity.z,6:0.00}");
        sb.AppendLine("---- Last 10 ----");

        // en yeni en üstte görünsün
        for (int i = list.Count - 1; i >= 0; i--)
        {
            var h = list[i];
            sb.AppendLine(
                $"{h.time,6:0.00}s  P({h.position.x,5:0.0},{h.position.y,5:0.0},{h.position.z,5:0.0}) " +
                $"V({h.velocity.x,5:0.0},{h.velocity.y,5:0.0},{h.velocity.z,5:0.0})"
            );
        }

        historyText.text = sb.ToString();
    }
}
