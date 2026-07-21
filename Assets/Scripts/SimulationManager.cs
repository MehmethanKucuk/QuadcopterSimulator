using System.Collections.Generic;
using UnityEngine;

public class SimulationManager : MonoBehaviour
{
    public DronePhysics dronePhysics;
    public DroneController droneController;

    [Header("Simulation Settings")]
    public bool isRunning = true;
    public float simulationDt = 0.01f;  // Integration step
    public float visualizeDt = 0.05f;   // 20 Hz HUD/History

    public enum IntegratorType { Euler, RungeKutta4 }
    public IntegratorType integrator = IntegratorType.RungeKutta4;

    public IReadOnlyList<DroneSample> GetHistory() => history;
    public DroneState GetState() => dronePhysics.state;


    [Header("HUD")]
    public DroneHUD hud;

    [Header("History")]
    public int maxHistoryCount = 10;

    float simTimeAccumulator = 0f;
    float vizTimeAccumulator = 0f;
    float simTime = 0f;

    List<DroneSample> history = new List<DroneSample>();

    Rigidbody droneRb;

    Vector3 pendingPos;
    Quaternion pendingRot;
    bool hasPendingPose = false;

    // Reset reference: state at the beginning of the game
    Vector3 startPos;
    Vector3 startEuler;

    void Start()
    {
        if (dronePhysics != null)
        {
            droneRb = dronePhysics.GetComponent<Rigidbody>();
            if (droneRb == null)
                Debug.LogWarning("The drone does not have a rigid body. A rigid body is required for collision protection!!.");

            // save the initial state
            startPos = dronePhysics.state.position;
            startEuler = dronePhysics.state.eulerAngles;

            pendingPos = startPos;
            pendingRot = Quaternion.Euler(startEuler);
            hasPendingPose = true;
        }
    }

    void Update()
    {
        if (!isRunning) return;

        float dt = Time.deltaTime;
        simTimeAccumulator += dt;
        vizTimeAccumulator += dt;

        while (simTimeAccumulator >= simulationDt)
        {
            StepSimulation();
            simTimeAccumulator -= simulationDt;
            simTime += simulationDt;
        }

        CachePendingPoseFromState();

        if (vizTimeAccumulator >= visualizeDt)
        {
            LogHistory();
            UpdateHUD();
            vizTimeAccumulator = 0f;
        }
    }

    void FixedUpdate()
    {
        if (!isRunning) return;
        if (!hasPendingPose) return;

        if (droneRb != null)
        {
            droneRb.MovePosition(pendingPos);
            droneRb.MoveRotation(pendingRot);
        }
        else
        {
            dronePhysics.transform.position = pendingPos;
            dronePhysics.transform.rotation = pendingRot;
        }
    }

    void StepSimulation()
    {
        ControlInput u = droneController.GetControlInput();
        DroneState s = dronePhysics.state;

        if (integrator == IntegratorType.Euler)
            dronePhysics.state = dronePhysics.EulerStep(s, u, simulationDt);
        else
            dronePhysics.state = dronePhysics.RungeKutta4Step(s, u, simulationDt);
    }

    void CachePendingPoseFromState()
    {
        var s = dronePhysics.state;
        pendingPos = s.position;
        pendingRot = Quaternion.Euler(s.eulerAngles);
        hasPendingPose = true;
    }

    void LogHistory()
    {
        var s = dronePhysics.state;
        history.Add(new DroneSample(simTime, s.position, s.velocity));
        if (history.Count > maxHistoryCount) history.RemoveAt(0);
    }

    void UpdateHUD()
    {
        if (hud != null)
            hud.UpdateHUD(dronePhysics.state, history);
    }

    // UI butons calls here from SimulationUI
    public void StartSimulation() => isRunning = true;
    public void StopSimulation()  => isRunning = false;

    public void ResetSimulation()
    {
        // reset simulation time and history.
        simTimeAccumulator = 0f;
        vizTimeAccumulator = 0f;
        simTime = 0f;
        history.Clear();

        // state reset
        DroneState rs = dronePhysics.state;
        rs.position = startPos;
        rs.velocity = Vector3.zero;
        rs.eulerAngles = startEuler;
        rs.angularVelocity = Vector3.zero;
        dronePhysics.state = rs;

        // pose buffer reset
        pendingPos = startPos;
        pendingRot = Quaternion.Euler(startEuler);
        hasPendingPose = true;

        // View
        if (droneRb != null)
        {
            droneRb.position = startPos;
            droneRb.rotation = Quaternion.Euler(startEuler);
            droneRb.linearVelocity = Vector3.zero;
            droneRb.angularVelocity = Vector3.zero;
        }
        else
        {
            dronePhysics.transform.position = startPos;
            dronePhysics.transform.rotation = Quaternion.Euler(startEuler);
        }

        // controller reset
        if (droneController != null)
            droneController.ResetAfterCrash(dronePhysics.state);
    }
}
