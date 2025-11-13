public class CustomRigidbody
{
    public Vec3 position;
    public Vec3 velocity;
    public Vec3 acceleration;
    public float mass = 1f;
    public bool useGravity = true;

    public CustomRigidbody(Vec3 startPos)
    {
        position = startPos;
    }

    public void AddForce(Vec3 force)
    {
        acceleration += force / mass;
    }

    public void Update(float deltaTime)
    {
        if (useGravity)
            AddForce(new Vec3(0, -9.81f * mass, 0));

        velocity += acceleration * deltaTime;
        position += velocity * deltaTime;
        acceleration = new Vec3(0, 0, 0); // reset after integration
    }
}
