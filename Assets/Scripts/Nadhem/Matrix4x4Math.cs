using UnityEngine;


namespace FractureSim
{
    /// <summary>
    /// Pure mathematics implementation of 4x4 matrix operations
    /// No Unity physics - manual transformation matrices only
    /// </summary>
    public static class Matrix4x4Math
    {
        /// <summary>
        /// Create a translation matrix
        /// [ 1  0  0  tx ]
        /// [ 0  1  0  ty ]
        /// [ 0  0  1  tz ]
        /// [ 0  0  0  1  ]
        /// </summary>
        public static float[,] CreateTranslation(Vector3 translation)
        {
            float[,] m = new float[4, 4];
            m[0, 0] = 1; m[0, 1] = 0; m[0, 2] = 0; m[0, 3] = translation.x;
            m[1, 0] = 0; m[1, 1] = 1; m[1, 2] = 0; m[1, 3] = translation.y;
            m[2, 0] = 0; m[2, 1] = 0; m[2, 2] = 1; m[2, 3] = translation.z;
            m[3, 0] = 0; m[3, 1] = 0; m[3, 2] = 0; m[3, 3] = 1;
            return m;
        }

        /// <summary>
        /// Create rotation matrix from quaternion
        /// Using quaternion to rotation matrix formula
        /// </summary>
        public static float[,] CreateRotation(Quaternion q)
        {
            float[,] m = new float[4, 4];
            
            float xx = q.x * q.x;
            float yy = q.y * q.y;
            float zz = q.z * q.z;
            float xy = q.x * q.y;
            float xz = q.x * q.z;
            float yz = q.y * q.z;
            float wx = q.w * q.x;
            float wy = q.w * q.y;
            float wz = q.w * q.z;

            m[0, 0] = 1 - 2 * (yy + zz); m[0, 1] = 2 * (xy - wz);     m[0, 2] = 2 * (xz + wy);     m[0, 3] = 0;
            m[1, 0] = 2 * (xy + wz);     m[1, 1] = 1 - 2 * (xx + zz); m[1, 2] = 2 * (yz - wx);     m[1, 3] = 0;
            m[2, 0] = 2 * (xz - wy);     m[2, 1] = 2 * (yz + wx);     m[2, 2] = 1 - 2 * (xx + yy); m[2, 3] = 0;
            m[3, 0] = 0;                 m[3, 1] = 0;                 m[3, 2] = 0;                 m[3, 3] = 1;
            
            return m;
        }

        /// <summary>
        /// Create rotation matrix around X axis
        /// </summary>
        public static float[,] CreateRotationX(float angleRadians)
        {
            float[,] m = new float[4, 4];
            float c = Mathf.Cos(angleRadians);
            float s = Mathf.Sin(angleRadians);
            
            m[0, 0] = 1; m[0, 1] = 0;  m[0, 2] = 0; m[0, 3] = 0;
            m[1, 0] = 0; m[1, 1] = c;  m[1, 2] = -s; m[1, 3] = 0;
            m[2, 0] = 0; m[2, 1] = s;  m[2, 2] = c; m[2, 3] = 0;
            m[3, 0] = 0; m[3, 1] = 0;  m[3, 2] = 0; m[3, 3] = 1;
            
            return m;
        }

        /// <summary>
        /// Create rotation matrix around Y axis
        /// </summary>
        public static float[,] CreateRotationY(float angleRadians)
        {
            float[,] m = new float[4, 4];
            float c = Mathf.Cos(angleRadians);
            float s = Mathf.Sin(angleRadians);
            
            m[0, 0] = c;  m[0, 1] = 0; m[0, 2] = s; m[0, 3] = 0;
            m[1, 0] = 0;  m[1, 1] = 1; m[1, 2] = 0; m[1, 3] = 0;
            m[2, 0] = -s; m[2, 1] = 0; m[2, 2] = c; m[2, 3] = 0;
            m[3, 0] = 0;  m[3, 1] = 0; m[3, 2] = 0; m[3, 3] = 1;
            
            return m;
        }

        /// <summary>
        /// Create rotation matrix around Z axis
        /// </summary>
        public static float[,] CreateRotationZ(float angleRadians)
        {
            float[,] m = new float[4, 4];
            float c = Mathf.Cos(angleRadians);
            float s = Mathf.Sin(angleRadians);
            
            m[0, 0] = c;  m[0, 1] = -s; m[0, 2] = 0; m[0, 3] = 0;
            m[1, 0] = s;  m[1, 1] = c;  m[1, 2] = 0; m[1, 3] = 0;
            m[2, 0] = 0;  m[2, 1] = 0;  m[2, 2] = 1; m[2, 3] = 0;
            m[3, 0] = 0;  m[3, 1] = 0;  m[3, 2] = 0; m[3, 3] = 1;
            
            return m;
        }

        /// <summary>
        /// Multiply two 4x4 matrices (composition of transformations)
        /// Result = A * B (apply B first, then A)
        /// </summary>
        public static float[,] Multiply(float[,] a, float[,] b)
        {
            float[,] result = new float[4, 4];
            
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    result[i, j] = 0;
                    for (int k = 0; k < 4; k++)
                    {
                        result[i, j] += a[i, k] * b[k, j];
                    }
                }
            }
            
            return result;
        }

        /// <summary>
        /// Transform a point by a 4x4 matrix
        /// </summary>
        public static Vector3 TransformPoint(float[,] matrix, Vector3 point)
        {
            float x = matrix[0, 0] * point.x + matrix[0, 1] * point.y + matrix[0, 2] * point.z + matrix[0, 3];
            float y = matrix[1, 0] * point.x + matrix[1, 1] * point.y + matrix[1, 2] * point.z + matrix[1, 3];
            float z = matrix[2, 0] * point.x + matrix[2, 1] * point.y + matrix[2, 2] * point.z + matrix[2, 3];
            float w = matrix[3, 0] * point.x + matrix[3, 1] * point.y + matrix[3, 2] * point.z + matrix[3, 3];
            
            return new Vector3(x / w, y / w, z / w);
        }

        /// <summary>
        /// Transform a direction (ignores translation)
        /// </summary>
        public static Vector3 TransformDirection(float[,] matrix, Vector3 direction)
        {
            float x = matrix[0, 0] * direction.x + matrix[0, 1] * direction.y + matrix[0, 2] * direction.z;
            float y = matrix[1, 0] * direction.x + matrix[1, 1] * direction.y + matrix[1, 2] * direction.z;
            float z = matrix[2, 0] * direction.x + matrix[2, 1] * direction.y + matrix[2, 2] * direction.z;
            
            return new Vector3(x, y, z);
        }

        /// <summary>
        /// Create identity matrix
        /// </summary>
        public static float[,] Identity()
        {
            float[,] m = new float[4, 4];
            m[0, 0] = 1; m[1, 1] = 1; m[2, 2] = 1; m[3, 3] = 1;
            return m;
        }

        /// <summary>
        /// Create a 3x3 rotation matrix from quaternion (for inertia tensor transformation)
        /// </summary>
        public static float[,] CreateRotation3x3(Quaternion q)
        {
            float[,] m = new float[3, 3];
            
            float xx = q.x * q.x;
            float yy = q.y * q.y;
            float zz = q.z * q.z;
            float xy = q.x * q.y;
            float xz = q.x * q.z;
            float yz = q.y * q.z;
            float wx = q.w * q.x;
            float wy = q.w * q.y;
            float wz = q.w * q.z;

            m[0, 0] = 1 - 2 * (yy + zz); m[0, 1] = 2 * (xy - wz);     m[0, 2] = 2 * (xz + wy);
            m[1, 0] = 2 * (xy + wz);     m[1, 1] = 1 - 2 * (xx + zz); m[1, 2] = 2 * (yz - wx);
            m[2, 0] = 2 * (xz - wy);     m[2, 1] = 2 * (yz + wx);     m[2, 2] = 1 - 2 * (xx + yy);
            
            return m;
        }

        /// <summary>
        /// Multiply 3x3 matrix with vector
        /// </summary>
        public static Vector3 Multiply3x3(float[,] m, Vector3 v)
        {
            return new Vector3(
                m[0, 0] * v.x + m[0, 1] * v.y + m[0, 2] * v.z,
                m[1, 0] * v.x + m[1, 1] * v.y + m[1, 2] * v.z,
                m[2, 0] * v.x + m[2, 1] * v.y + m[2, 2] * v.z
            );
        }

        /// <summary>
        /// Multiply two 3x3 matrices
        /// </summary>
        public static float[,] Multiply3x3(float[,] a, float[,] b)
        {
            float[,] result = new float[3, 3];
            
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    result[i, j] = 0;
                    for (int k = 0; k < 3; k++)
                    {
                        result[i, j] += a[i, k] * b[k, j];
                    }
                }
            }
            
            return result;
        }

        /// <summary>
        /// Transpose a 3x3 matrix
        /// </summary>
        public static float[,] Transpose3x3(float[,] m)
        {
            float[,] result = new float[3, 3];
            
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    result[i, j] = m[j, i];
                }
            }
            
            return result;
        }
    }
}