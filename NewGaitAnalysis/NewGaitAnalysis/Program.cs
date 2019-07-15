using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using Microsoft.Kinect;

namespace NewGaitAnalysis
{
    using JointsList = List<Dictionary<JointType, Joint>>;

    class Program
    {
        static void Main(string[] args)
        {
            //Console.WriteLine("hello, to fudido, mas vou conseguir!");

            while (true)
            {
                string input = ShowDirectories();

                string folderName = input.Split('\\')[input.Split('\\').Length - 1];
                Console.WriteLine(folderName);

                JointsList jointsList = new JointsList();

                string file1 = input + @"\localkinect-data.txt";
                string file2 = input + @"\netkinect-data.txt";

                string[] lines1 = File.ReadAllLines(file1);
                string[] lines2 = File.ReadAllLines(file2);

                var prevJoints1 = SkeletonInterpolation.GetUntrackedSkeleton();
                var prevJoints2 = SkeletonInterpolation.GetUntrackedSkeleton();

                for (int i = 0; i < lines1.Length; i++)
                {
                    //var jointsFrame = Parser.BuildJointsAndJointPoints(line);
                    var joints1 = Parser.BuildJointsAndJointPoints(lines1[i]).Item1;
                    var joints2 = Parser.BuildJointsAndJointPoints(lines2[i]).Item1;

                    var interSkeleton = SkeletonInterpolation.InterpolatedBody(joints1, joints2, prevJoints1, prevJoints2);

                    jointsList.Add(interSkeleton);
                    prevJoints1 = joints1;
                    prevJoints2 = joints2;
                }

                GaitAnalysis gait = new GaitAnalysis(jointsList);
                WriteCSV(gait.FootDistances, filename: folderName);
            }
        }

        static string ShowDirectories(string path = @"C:\Users\jrobe\Documents\SVR2019\gravacoes")
        {
            string[] filePaths = Directory.GetDirectories(path);

            int i = 0;
            foreach (string aPath in filePaths)
            {
                string[] tmp = aPath.Split('\\');
                string dir = tmp[tmp.Length - 1];
                Console.WriteLine("  " + i + ". " + dir);
                i += 1;
            }

            int input = int.Parse(Console.ReadLine());

            return filePaths[input];
        }

        static void WriteCSV(List<float> values, string filename = "output")
        {
            List<string> lines = new List<string>();
            int i = 0;
            foreach (float value in values)
            {
                lines.Add(String.Format("{0},{1}", i, (value*100).ToString().Replace(',', '.')));
                i += 1;
            }

            File.WriteAllLines(filename + ".csv", lines);
        }
    }
}
