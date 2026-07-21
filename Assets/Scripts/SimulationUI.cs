using TMPro;
using UnityEngine;
using UnityEngine.UI;


//Star/Stop/Reset Panels
public class SimulationUI : MonoBehaviour
{
    [Header("Refs")]
    public SimulationManager sim;

    [Header("Buttons")]
    public Button startButton;
    public Button stopButton;
    public Button resetButton;

    [Header("UI Text")]
    public TMP_Text statusText;

    void Start()
    {
        
        if (sim == null) sim = FindFirstObjectByType<SimulationManager>();

        // Button click events
        if (startButton != null) startButton.onClick.AddListener(OnStart);
        if (stopButton != null) stopButton.onClick.AddListener(OnStop);
        if (resetButton != null) resetButton.onClick.AddListener(OnReset);

        RefreshUI();
    }

    void OnStart()
    {
        if (sim != null) sim.StartSimulation();
        RefreshUI();
    }

    void OnStop()
    {
        if (sim != null) sim.StopSimulation();
        RefreshUI();
    }

    void OnReset()
    {
        if (sim != null) sim.ResetSimulation();
        // After reset automatically stays in "Stop":
        if (sim != null) sim.StopSimulation();
        RefreshUI();
    }

    void RefreshUI()
    {
        bool running = (sim != null && sim.isRunning);

        if (statusText != null)
            statusText.text = running ? "RUNNING" : "STOPPED";

        if (startButton != null) startButton.interactable = !running;
        if (stopButton != null)  stopButton.interactable  = running;

        if (resetButton != null) resetButton.interactable = true;
    }
}
