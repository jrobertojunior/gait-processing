﻿using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using Microsoft.Kinect;
using System.Windows;

namespace NewGaitAnalysis
{
    public static class Parser
    {
        public static string BuildMessage(Dictionary<JointType, Joint> joints)
        {
            string message = "";
            foreach (JointType jointType in joints.Keys)
            {
                CameraSpacePoint position = joints[jointType].Position;

                string info = string.Format("{0} {1} {2} {3} ", (int)joints[jointType].TrackingState, joints[jointType].Position.X, joints[jointType].Position.Y, joints[jointType].Position.Z);
                message += info;
            }

            return message;
        }

        public static string BuildMessage(Dictionary<JointType, Joint> joints, Dictionary<JointType, Point> jointPoints)
        {
            string message = "";
            foreach (JointType jointType in joints.Keys)
            {
                CameraSpacePoint position = joints[jointType].Position;

                string info = string.Format("{0} {1} {2} {3} {4} {5} ", (int)joints[jointType].TrackingState,
                    joints[jointType].Position.X, joints[jointType].Position.Y, joints[jointType].Position.Z,
                    jointPoints[jointType].X, jointPoints[jointType].Y);

                message += info;
            }

            return message;
        }

        public static Dictionary<JointType, Joint> BuildJoints(string message)
        {
            string[] words = message.Split(' ');

            Dictionary<JointType, Joint> joints = new Dictionary<JointType, Joint>();

            int i = 0;
            foreach (JointType jointType in Enum.GetValues(typeof(JointType)))
            {
                Joint joint = new Joint()
                {
                    JointType = jointType,
                    TrackingState = (TrackingState)int.Parse(words[i]),
                    Position = new CameraSpacePoint()
                    {
                        X = float.Parse(words[i + 1]),
                        Y = float.Parse(words[i + 2]),
                        Z = float.Parse(words[i + 3])
                    }
                };

                joints.Add(jointType, joint);

                i += 4;
            }

            return joints;
        }

        public static Tuple<Dictionary<JointType, Joint>, Dictionary<JointType, Point>> BuildJointsAndJointPoints(string message)
        {
            string[] words = message.Split(' ');

            Dictionary<JointType, Joint> joints = new Dictionary<JointType, Joint>();
            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

            int i = 0;
            foreach (JointType jointType in Enum.GetValues(typeof(JointType)))
            {
                Joint joint = new Joint();

                joint.JointType = jointType;
                joint.TrackingState = (TrackingState)int.Parse(words[i]);
                joint.Position = new CameraSpacePoint();

                joint.Position.X = StringToFloat(words[i + 1]);
                joint.Position.Y = StringToFloat(words[i + 2]);
                joint.Position.Z = StringToFloat(words[i + 3]);

                joints.Add(jointType, joint);

                Point point = new Point()
                {
                    X = StringToFloat(words[i + 4]),
                    Y = StringToFloat(words[i + 5])
                };

                jointPoints.Add(jointType, point);

                i += 6;
            }

            return new Tuple<Dictionary<JointType, Joint>, Dictionary<JointType, Point>>(joints, jointPoints);
        }

        static float StringToFloat(string s)
        {
            if (s.Contains("E"))
            {
                string[] parts = s.Split('E');
                int expoent = int.Parse(parts[1]);
                //Console.WriteLine(expoent);
                float number = (float)Convert.ToDecimal(parts[0]);
                number = (float)Math.Pow((double)number, (double)expoent);
                return number;
            }
            else if (s == "-∞")
            {
                return float.NegativeInfinity;
            }

            return (float)System.Convert.ToDecimal(s);
        }
    }
}
