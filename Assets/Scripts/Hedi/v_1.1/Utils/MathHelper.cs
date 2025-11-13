using UnityEngine;

namespace CustomPhysics.v_1_1.Utils
{
    public static class MathHelper
    {
        public static Vector3 TransformPoint(Vector3 point, Vector3 position, Quaternion rotation, Vector3 scale)
        {
            Matrix4x4 matrix = Matrix4x4.TRS(position, rotation, scale);
            return matrix.MultiplyPoint3x4(point);
        }
        
        public static Vector3 TransformDirection(Vector3 direction, Quaternion rotation)
        {
            return rotation * direction;
        }
        
        public static float Clamp(float value, float min, float max)
        {
            return value < min ? min : value > max ? max : value;
        }
        
        public static Vector3 ClampMagnitude(Vector3 vector, float maxLength)
        {
            if (vector.sqrMagnitude > maxLength * maxLength)
            {
                return vector.normalized * maxLength;
            }
            return vector;
        }
        
        public static bool Approximately(float a, float b, float tolerance = 0.0001f)
        {
            return Mathf.Abs(a - b) < tolerance;
        }
        
        public static bool Approximately(Vector3 a, Vector3 b, float tolerance = 0.0001f)
        {
            return Approximately(a.x, b.x, tolerance) &&
                   Approximately(a.y, b.y, tolerance) &&
                   Approximately(a.z, b.z, tolerance);
        }
    }
}