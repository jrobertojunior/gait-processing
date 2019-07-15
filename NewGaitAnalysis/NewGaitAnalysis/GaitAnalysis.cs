using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;


namespace NewGaitAnalysis
{
    using JointsList = List<Dictionary<JointType, Joint>>;

    class GaitAnalysis
    {
        private Dictionary<JointType, Joint> prevJoints;

        public List<Dictionary<JointType, Joint>> JointsList;

        public List<float> FootDistances { get; private set; }
        public List<CameraSpacePoint> SpineBasePositions { get; private set; }

        public List<float> leftAnkledistancesFromPlane { get; private set; }
        public List<float> rightAnkledistancesFromPlane { get; private set; }

        public GaitAnalysis(List<Dictionary<JointType, Joint>> jointsList)
        {
            JointsList = jointsList;
            GetSpineCenterPositions(jointsList);
            Analyze(jointsList);
        }

        public void Analyze(JointsList jointsList)
        {
            FootDistances = GetAnkleDistancesFromEachOther(jointsList);
            SpineBasePositions = GetSpineCenterPositions(jointsList);

            leftAnkledistancesFromPlane = new List<float>();
            rightAnkledistancesFromPlane = new List<float>();
            foreach (Dictionary<JointType, Joint> joints in jointsList)
            {
                Planes planes = GetPlanes(joints);

                float distLeft = GetDistanceFromPlane(joints[JointType.AnkleLeft].Position, joints[JointType.SpineBase].Position, planes.FrontalPlane);
                float distRight = GetDistanceFromPlane(joints[JointType.AnkleRight].Position, joints[JointType.SpineBase].Position, planes.FrontalPlane);
                leftAnkledistancesFromPlane.Add(distLeft);
                rightAnkledistancesFromPlane.Add(distRight);
            }
        }

        private List<float> GetAnkleDistancesFromEachOther(JointsList jointsList)
        {
            var footDistances = new List<float>();

            foreach (var joints in jointsList)
            {
                CameraSpacePoint leftAnkle = joints[JointType.AnkleLeft].Position;
                CameraSpacePoint rightAnkle = joints[JointType.AnkleRight].Position;

                float footDist = LinearAlgebra.DistanceBetweenTwoPoints(leftAnkle, rightAnkle);

                footDistances.Add(footDist);
            }

            return footDistances;
        }

        public List<CameraSpacePoint> GetSpineCenterPositions(JointsList jointsList)
        {
            var spineBasePositions = new List<CameraSpacePoint>();

            foreach (var joints in jointsList)
            {
                CameraSpacePoint pos = joints[JointType.SpineBase].Position;
                //Console.WriteLine(String.Format("{0} {1} {2}", pos.X, pos.Y, pos.Z));
                spineBasePositions.Add(pos);
            }
            return spineBasePositions;
        }

        public static float GetDistanceFromPlane(CameraSpacePoint point, CameraSpacePoint planePos, CameraSpacePoint normal)
        {
            float dist = LinearAlgebra.DotProduct(normal, (LinearAlgebra.Sub(point, planePos)));
            return dist;
        }


        public static Planes GetPlanes(Dictionary<JointType, Joint> joints)
        {
            CameraSpacePoint spineBase = joints[JointType.SpineBase].Position;
            CameraSpacePoint spineMid = joints[JointType.SpineMid].Position;

            CameraSpacePoint horizontalPlane =LinearAlgebra.Normalize(LinearAlgebra.Sub(spineMid, spineBase));
            CameraSpacePoint hipToHip = LinearAlgebra.Normalize( LinearAlgebra.Sub(joints[JointType.HipLeft].Position, joints[JointType.HipRight].Position)); // difference between both hips
            CameraSpacePoint sagittalPlane = LinearAlgebra.Normalize(LinearAlgebra.CrossProduct(horizontalPlane, hipToHip));
            CameraSpacePoint frontalPlane = LinearAlgebra.Normalize(LinearAlgebra.CrossProduct(horizontalPlane, sagittalPlane));


            Planes planes = new Planes(horizontalPlane, sagittalPlane, frontalPlane);

            return planes;
        }
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
