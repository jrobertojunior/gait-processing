using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;


namespace NewGaitAnalysis
{
    using JointsList = List<Dictionary<JointType, Joint>>;

    public enum FootPhase
    {
        Swing = 0,
        Stance
    }

    class GaitAnalysis
    {
        public int Cycles { get; private set; }

        private Dictionary<JointType, Joint> prevJoints;

        public Foot LeftFoot;
        public Foot RightFoot;

        public List<Dictionary<JointType, Joint>> JointsList;

        public List<float> FootDistances { get; private set; }

        public GaitAnalysis()
        {
            LeftFoot = new Foot();
            RightFoot = new Foot();
        }

        public GaitAnalysis(List<Dictionary<JointType, Joint>> jointsList)
        {
            JointsList = jointsList;

            Analyze(jointsList);
        }

        public void Analyze(JointsList jointsList)
        {
            FootDistances = new List<float>();
            float maxValue = 0.0f;

            foreach (var joint in jointsList)
            {
                CameraSpacePoint leftAnkle = joint[JointType.AnkleLeft].Position;
                CameraSpacePoint rightAnkle = joint[JointType.AnkleRight].Position;

                float footDist = LinearAlgebra.DistanceBetweenTwoPoints(leftAnkle, rightAnkle);

                if (footDist > maxValue)
                {
                    maxValue = footDist;
                }

                FootDistances.Add(footDist);
            }

            //Console.WriteLine("Max foot distance: " + maxValue);
        }

        public void Update(Dictionary<JointType, Joint> joints)
        {
            int trackedJoints = 0;
            foreach (JointType jointType in joints.Keys)
            {
                if (joints[jointType].TrackingState == TrackingState.Tracked || joints[jointType].TrackingState == TrackingState.Inferred)
                {
                    trackedJoints += 1;
                }
            }
            //Console.WriteLine(Enum.GetNames(typeof(JointType)).Length);
            float confidence = trackedJoints / Enum.GetNames(typeof(JointType)).Length; // ratio of tracked-joints and total-joints
            confidence = 1;
            if (confidence > 0.8)
            {
                LeftFoot.Update(joints[JointType.FootLeft].Position);
            }
        }
    }

    class Foot
    {
        public FootPhase Phase { get; private set; }
        public float Velocity { get; private set; }
        public float Acceleration { get; private set; }

        private CameraSpacePoint previousPosition;
        private CameraSpacePoint Position;
        private float previousVelocity;
        private float previousAcceleration;

        private List<float> VelocityHistory;
        private List<float> AccelerationHistory;

        private int historyLen;
        private float phaseThreshold;

        public Foot()
        {
            historyLen = 5;
            phaseThreshold = 0.15f;

            Phase = FootPhase.Stance;

            Position = new CameraSpacePoint
            {
                X = 0,
                Y = 0,
                Z = 0
            };
            Velocity = 0.0f;
            Acceleration = 0.0f;

            VelocityHistory = new List<float>();
            AccelerationHistory = new List<float>();
        }

        public void Update(CameraSpacePoint position)
        {
            // Update call stack:
            // Update(position) >> UpdateVelocity >> UpdateAcceleration >> UpdatePhase

            previousPosition = Position; // update previous value
            Position = position; // update current value

            if (!IsZero(previousPosition))
            {
                UpdateVelocity(Position, previousPosition);
            }
        }

        private void UpdateVelocity(CameraSpacePoint currentPos, CameraSpacePoint previousPos)
        {
            previousVelocity = Velocity; // update previous value

            float v = LinearAlgebra.DistanceBetweenTwoPoints(currentPos, previousPos); // update current value

            if (v < 0.6)
            {
                Velocity = v;
            }


            AddValue(Velocity, VelocityHistory); // update history

            if (previousVelocity != 0.0f)
            {
                UpdateAcceleration(Velocity, previousVelocity);

                //UpdatePhase(Velocity);
            }
        }

        private void UpdateAcceleration(float currentVel, float previousVel)
        {
            previousAcceleration = Acceleration; // update previous value
            Acceleration = currentVel - previousVel; // update current value

            AccelerationHistory = AddValue(Acceleration, AccelerationHistory); // update history

            //UpdatePhase(Acceleration);
        }

        private void UpdatePhase(float currentAccel)
        {
            if (currentAccel > phaseThreshold)
            {
                Phase = FootPhase.Swing;
            }
            else
            {
                Phase = FootPhase.Stance;
            }

            Console.WriteLine(Phase);
        }

        private List<float> AddValue(float value, List<float> list)
        {
            if (list.Capacity < historyLen)
            {
                list.Add(value);
            }
            else
            {
                for (int i = 0; i < historyLen - 1; i++)
                {
                    list[i] = list[i + 1];
                }

                list[historyLen - 1] = value;
            }

            return list;
        }

        private bool IsZero(CameraSpacePoint pos)
        {
            if (pos.X == 0 && pos.Y == 0 && pos.Z == 0) return true;

            return false;
        }
    }
}
