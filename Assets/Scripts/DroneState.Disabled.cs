using UnityEngine;

[System.Serializable]
public class DroneState
{
    public Vector3 position;
    public Vector3 velocity;
    public Vector3 eulerAngles;      // degrees
    public Vector3 angularVelocity;  // degrees/sec (senin sistemin böyle kullanıyor)

    public DroneState() { }

    public DroneState(Vector3 pos, Vector3 vel, Vector3 eulerDeg, Vector3 angVelDeg)
    {
        position = pos;
        velocity = vel;
        eulerAngles = eulerDeg;
        angularVelocity = angVelDeg;
    }

    public DroneState Clone()
    {
        return new DroneState(position, velocity, eulerAngles, angularVelocity);
    }
}
