using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace NewGaitAnalysis
{
    public static class SkeletonInterpolation
    {
        public static Dictionary<JointType, Joint> InterpolatedBody(Dictionary<JointType, Joint> joints1, Dictionary<JointType, Joint> prevJoints1, Dictionary<JointType, Joint> prevJoints2, Dictionary<JointType, Joint> joints2, int nTrackedJointsThreshold = 15, float changeInMotionThreshold = 20)
        {
            Dictionary<JointType, Joint> interBody = new Dictionary<JointType, Joint>();

            //if (IsValidSkeleton(joints1, prevJoints1, nTrackedJointsThreshold, changeInMotionThreshold) && IsValidSkeleton(joints2, prevJoints2, nTrackedJointsThreshold, changeInMotionThreshold))
            //{
            //    foreach (JointType type in Enum.GetValues(typeof(JointType)))
            //    {
            //        CameraSpacePoint interPos;
            //        if (LinearAlgebra.DistanceBetweenTwoPoints(joints1[type].Position, joints2[type].Position) > 0.2)
            //        {
            //            if (NumberOfTrackeJoints(joints1) > NumberOfTrackeJoints(joints2))
            //            {
            //                interPos = joints1[type].Position;
            //            }
            //            else
            //            {
            //                interPos = joints2[type].Position;
            //            }
            //        }
            //        else
            //        {
            //            interPos = PointInterpolation(joints1[type].Position, joints2[type].Position);
            //        }
            //        Joint interJoint = new Joint
            //        {
            //            JointType = type,
            //            TrackingState = TrackingState.Tracked,
            //            Position = interPos
            //        };
            //        interBody.Add(type, interJoint);
            //    }
            //}
            /*else*/ if (IsValidSkeleton(joints2, prevJoints2, nTrackedJointsThreshold, changeInMotionThreshold))
            {
                interBody = joints2;
            }
            else if (IsValidSkeleton(joints1, prevJoints1, nTrackedJointsThreshold, changeInMotionThreshold))
            {
                interBody = joints1;
            }
            else
            {
                interBody = GetUntrackedSkeleton();
            }
            return interBody;
        }

        private static bool IsValidSkeleton(Dictionary<JointType, Joint> joints, Dictionary<JointType, Joint> prevJoints, int nTrackedJointsThreshold, float changeInMotionThreshold)
        {
            if (NumberOfTrackeJoints(joints) > nTrackedJointsThreshold && ChangeInMotion(joints, prevJoints) < changeInMotionThreshold)
            {
                return true;
            }
            return false;
        }

        private static float ChangeInMotion(Dictionary<JointType, Joint> currentJoints, Dictionary<JointType, Joint> prevJoints)
        {
            float totalDistance = 0;
            foreach (JointType type in Enum.GetValues(typeof(JointType)))
            {
                totalDistance += LinearAlgebra.DistanceBetweenTwoPoints(currentJoints[type].Position, prevJoints[type].Position);
            }
            float result = totalDistance / 25;
            return result;
        }

        private static int NumberOfTrackeJoints(Dictionary<JointType, Joint> joints)
        {
            int nTrackedJonits = 0;
            foreach (Joint joint in joints.Values)
            {
                if (joint.TrackingState == TrackingState.Tracked)
                {
                    nTrackedJonits += 1;
                }
            }
            return nTrackedJonits;
        }

        private static CameraSpacePoint PointInterpolation(CameraSpacePoint a, CameraSpacePoint b)
        {
            return new CameraSpacePoint
            {
                X = (a.X + b.X) / 2,
                Y = (a.Y + b.Y) / 2,
                Z = (a.Z + b.Z) / 2
            };
        }

        public static Dictionary<JointType, Joint> GetUntrackedSkeleton()
        {
            Dictionary<JointType, Joint> joints = new Dictionary<JointType, Joint>();

            foreach (JointType jointType in Enum.GetValues(typeof(JointType)))
            {
                Joint joint = new Joint
                {
                    TrackingState = TrackingState.NotTracked,
                    JointType = jointType,
                    Position = new CameraSpacePoint
                    {
                        X = 0f,
                        Y = 0f,
                        Z = 0f
                    }
                };

                joints.Add(jointType, joint);
            }

            return joints;
        }
    }
}
