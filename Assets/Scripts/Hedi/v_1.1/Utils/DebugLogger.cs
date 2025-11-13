using UnityEngine;

namespace CustomPhysics.v_1_1.Utils
{
    public static class DebugLogger
    {
        public static void LogCollision(string colliderA, string colliderB, Vector3 contactPoint)
        {
            Debug.Log($"Collision: {colliderA} <-> {colliderB} at {contactPoint}");
        }
        
        public static void LogPhysics(string message)
        {
            Debug.Log($"[Physics] {message}");
        }
        
        public static void LogWarning(string message)
        {
            Debug.LogWarning($"[Physics] {message}");
        }
        
        public static void LogError(string message)
        {
            Debug.LogError($"[Physics] {message}");
        }
        
        public static void DrawDebugRay(Vector3 origin, Vector3 direction, Color color, float duration = 0.1f)
        {
            Debug.DrawRay(origin, direction, color, duration);
        }
        
        public static void DrawDebugPoint(Vector3 point, Color color, float size = 0.1f, float duration = 0.1f)
        {
            Debug.DrawLine(point - Vector3.right * size, point + Vector3.right * size, color, duration);
            Debug.DrawLine(point - Vector3.up * size, point + Vector3.up * size, color, duration);
            Debug.DrawLine(point - Vector3.forward * size, point + Vector3.forward * size, color, duration);
        }
    }
}