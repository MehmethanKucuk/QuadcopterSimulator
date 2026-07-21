using UnityEngine;

// History listesinde tutacağımız küçük veri paketi
public class DroneSample
{
    public float time;
    public Vector3 position;
    public Vector3 velocity;

    public DroneSample(float time, Vector3 pos, Vector3 vel)
    {
        this.time = time;
        this.position = pos;
        this.velocity = vel;
    }
}
