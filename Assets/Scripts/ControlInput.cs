using UnityEngine;

// Control inputs: total thrust and moments
public struct ControlInput
{
    public float totalThrust;   // Total upward thrust
    public Vector3 moments;     // Pitch, Yaw, Roll moments
}
