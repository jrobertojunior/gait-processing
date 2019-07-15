using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace NewGaitAnalysis
{
    static class LinearAlgebra
    {
        public static float DistanceBetweenTwoPoints(CameraSpacePoint point1, CameraSpacePoint point2)
        {
            return (float)Math.Sqrt(Math.Pow(point1.X - point2.X, 2) + Math.Pow(point1.Y - point2.Y, 2) + Math.Pow(point1.Z - point2.Z, 2));
        }

        public static float Norm(CameraSpacePoint vector)
        {
            return (float)Math.Sqrt(Math.Pow(vector.X, 2) + Math.Pow(vector.Y, 2) + Math.Pow(vector.Z, 2));
        }

        public static CameraSpacePoint Normalize(CameraSpacePoint vector)
        {
            float norm = Norm(vector);
            CameraSpacePoint result;

            result.X = vector.X / norm;
            result.Y = vector.Y / norm;
            result.Z = vector.Z / norm;

            return result;
        }

        public static float AngleBetweenTwoVectors(CameraSpacePoint vector1, CameraSpacePoint vector2)
        {
            float norm1 = Norm(vector1);
            float norm2 = Norm(vector2);

            if (norm1 == 0 || norm2 == 0) return 0;

            float _dotProduct = DotProduct(vector1, vector2);

            return (float)(Math.Acos(_dotProduct / (norm1 * norm2)) * (180 / 3.1415));
        }

        public static float DotProduct(CameraSpacePoint vector1, CameraSpacePoint vector2)
        {
            return (vector1.X * vector2.X) + (vector1.Y * vector2.Y) + (vector1.Z * vector2.Z);
        }

        public static CameraSpacePoint CrossProduct(CameraSpacePoint vector1, CameraSpacePoint vector2)
        {
            CameraSpacePoint result;

            result.X = (vector1.Y * vector2.Z) - (vector1.Z * vector2.Y);
            result.Y = (vector1.Z * vector2.X) - (vector1.X * vector2.Z);
            result.Z = (vector1.X * vector2.Y) - (vector1.Y * vector2.X);

            return result;
        }

        public static CameraSpacePoint Sub(CameraSpacePoint vector1, CameraSpacePoint vector2)
        {
            return new CameraSpacePoint
            {
                X = vector1.X - vector2.X,
                Y = vector1.Y - vector2.Y,
                Z = vector1.Z - vector2.Z
            };
        }

        public static CameraSpacePoint Scale(CameraSpacePoint vector, float scaler)
        {
            return new CameraSpacePoint
            {
                X = vector.X * scaler,
                Y = vector.Y * scaler,
                Z = vector.Z * scaler
            };
        }

        public static CameraSpacePoint Transform(CameraSpacePoint vector, CameraSpacePoint transformVector)
        {
            return new CameraSpacePoint
            {
                X = vector.X + transformVector.X,
                Y = vector.Y + transformVector.Y,
                Z = vector.Z + transformVector.Z
            };
        }

        /// <summary>
        /// The projection of vector2 onto vector1
        /// </summary>
        /// <param name="vector1"></param>
        /// <param name="vector2"></param>
        /// <returns></returns>
        public static CameraSpacePoint VectorProjection(CameraSpacePoint vector1, CameraSpacePoint vector2)
        {
            return Scale(Normalize(vector1), ScalarProjection(vector1, vector2));
        }

        /// <summary>
        /// The scalar projection of vector2 onto the direction of vector1
        /// </summary>
        /// <param name="vector1"></param>
        /// <param name="vector2"></param>
        /// <returns></returns>
        public static float ScalarProjection(CameraSpacePoint vector1, CameraSpacePoint vector2)
        {
            return DotProduct(vector1, vector2) / Norm(vector1);
        }

        //public static CameraSpacePoint PointProjectionIntoPlane(CameraSpacePoint point, CameraSpacePoint planeNormal)
        //{
        //    float x = point.X;
        //    float y = point.Y;
        //    float z = point.Z;

        //    float a = planeNormal.X;
        //    float b = planeNormal.Y;
        //    float c = planeNormal.Z;

        //    float t = (a*)
        //}

        public static Planes GetPlanes(Dictionary<JointType, Joint> joints)
        {
            CameraSpacePoint spineBase = joints[JointType.SpineBase].Position;
            CameraSpacePoint spineMid = joints[JointType.SpineMid].Position;

            CameraSpacePoint horizontalPlane = LinearAlgebra.Sub(spineMid, spineBase);
            CameraSpacePoint hipToHip = LinearAlgebra.Sub(joints[JointType.HipLeft].Position, joints[JointType.HipRight].Position); // difference between both hips
            CameraSpacePoint sagittalPlane = LinearAlgebra.CrossProduct(horizontalPlane, hipToHip);
            CameraSpacePoint frontalPlane = LinearAlgebra.CrossProduct(horizontalPlane, sagittalPlane);


            Planes plane = new Planes(horizontalPlane, sagittalPlane, frontalPlane);

            return new Planes(horizontalPlane, sagittalPlane, frontalPlane); ;
        }

        public struct Planes
        {
            public CameraSpacePoint HorizontalPlane;
            public CameraSpacePoint SagittalPlane;
            public CameraSpacePoint FrontalPlane;

            public Planes(CameraSpacePoint horizontalPlane, CameraSpacePoint sagittalPlane, CameraSpacePoint frontalPlane)
            {
                HorizontalPlane = horizontalPlane;
                SagittalPlane = sagittalPlane;
                FrontalPlane = frontalPlane;
            }
        }
    }
}