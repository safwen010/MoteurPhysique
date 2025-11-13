using System.Collections.Generic;

public class SimplePhysics
{
    public List<CustomRigidbody> bodies = new List<CustomRigidbody>();
    public List<BoxCollider3D> colliders = new List<BoxCollider3D>();

    public void Add(CustomRigidbody rb, BoxCollider3D col)
    {
        bodies.Add(rb);
        colliders.Add(col);
    }

    public void Step(float deltaTime)
    {
        for (int i = 0; i < bodies.Count; i++)
        {
            bodies[i].Update(deltaTime);
            colliders[i].center = bodies[i].position;
        }

        // basic collision handling (AABB)
        for (int i = 0; i < colliders.Count; i++)
        {
            for (int j = i + 1; j < colliders.Count; j++)
            {
                if (colliders[i].Intersects(colliders[j]))
                {
                    // simple collision response (reverse velocity)
                    bodies[i].velocity = bodies[i].velocity * -0.5f;
                    bodies[j].velocity = bodies[j].velocity * -0.5f;
                }
            }
        }
    }
}
